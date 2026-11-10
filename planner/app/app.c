#include "app/app.h"
#include "planning/brush_model.h"
#include "config.h"
#include "hal/hal.h"
#include "motion/motion.h"
#include "vision/palette.h"
#include "planning/planner.h"
#include "vision/vision.h"
#include <stdio.h>
#include <stdlib.h>

static const int k_layer_brush_id[NUM_LAYERS] = LAYER_BRUSH_ID;

/* calibration strip layout, canvas mm */
#define CAL_LINES_PER_BRUSH 6
#define CAL_FIRST_ROW_MM    100.0f
#define CAL_ROW_SPACING_MM  45.0f
#define CAL_LINE_START_MM   100.0f
#define CAL_LINE_END_MM     400.0f

static void fatal(const char *reason)
{
    hal_log("FATAL: %s\n", reason);
    motion_park();
    exit(1);
}

/* Write a preview image (sim_out/<name>.ppm on the host). */
static void dump(const char *name, const lab_image_t *image)
{
    uint8_t *rgb = malloc((size_t)image->width * image->height * 3);
    if (!rgb)
    {
        return;
    }
    img_to_rgb8(image, rgb);
    hal_debug_dump_rgb(name, image->width, image->height, rgb);
    free(rgb);
}

/* Photograph and rectify. Brush is parked first (motion_park waits with M400)
 * so the gantry is out of frame. */
static bool capture(capture_purpose_t purpose, lab_image_t *rectified)
{
    if (purpose == CAPTURE_CANVAS && !motion_park())
    {
        return false;
    }
    cam_frame_t frame;
    if (!hal_camera_capture(purpose, &frame))
    {
        return false;
    }
    bool ok = vision_rectify(&frame, rectified);
    hal_camera_release(&frame);
    return ok;
}

static float cal_row_mm(int brush, int line)
{
    return CAL_FIRST_ROW_MM + (brush * CAL_LINES_PER_BRUSH + line) * CAL_ROW_SPACING_MM;
}

/* ---- brush calibration: paint lines at known depths, measure, fit ---- */
static void calibrate_brushes(brush_set_t *brushes)
{
    float depth_mm[NUM_BRUSHES][CAL_LINES_PER_BRUSH];
    hal_wait_button("CAL: lay a scrap canvas with magenta corner dots on the bed, "
                    "fill pot 0 with dark paint, load brush 0, press button");
    for (int brush = 0; brush < NUM_BRUSHES; brush++)
    {
        if (brush > 0)
        {
            motion_rinse();
            motion_park();
            char prompt[64];
            snprintf(prompt, sizeof prompt, "CAL: swap to brush %d, press button", brush);
            hal_wait_button(prompt);
        }
        for (int line = 0; line < CAL_LINES_PER_BRUSH; line++)
        {
            if (line % 2 == 0)
            {
                motion_load_paint(0, false);
            }
            depth_mm[brush][line] = (line + 0.5f) * Z_PRESS_MAX_MM / CAL_LINES_PER_BRUSH;
            float row_mm = cal_row_mm(brush, line);
            motion_paint_line(CAL_LINE_START_MM, row_mm, CAL_LINE_END_MM, row_mm, depth_mm[brush][line],
                              &brushes->brush[brush]);
        }
    }
    motion_rinse();

    lab_image_t photo;
    if (!img_alloc(&photo, WORK_RES, WORK_RES))
    {
        fatal("oom (cal)");
    }
    if (!capture(CAPTURE_CANVAS, &photo))
    {
        hal_log("CAL: no canvas photo - keeping previous brush model\n");
        img_free(&photo);
        return;
    }
    dump("calibration", &photo);
    lab_t background = img_get(&photo, (int)(700 / MM_PER_PX), (int)(100 / MM_PER_PX)); /* unpainted area */
    for (int brush = 0; brush < NUM_BRUSHES; brush++)
    {
        float width_mm[CAL_LINES_PER_BRUSH];
        for (int line = 0; line < CAL_LINES_PER_BRUSH; line++)
        {
            int row_px = (int)(cal_row_mm(brush, line) / MM_PER_PX);
            /* measure the middle of the line, away from the tapered ends */
            width_mm[line] = brush_measure_width(&photo, background, row_px, (int)(150 / MM_PER_PX),
                                                 (int)(350 / MM_PER_PX));
            hal_log("CAL: brush %d depth %.2f mm -> width %.1f mm\n", brush, depth_mm[brush][line],
                    width_mm[line]);
        }
        brush_t *model = &brushes->brush[brush];
        float r_squared = brush_fit(model, depth_mm[brush], width_mm, CAL_LINES_PER_BRUSH);
        hal_log("CAL: brush %d  width = %.2f + %.2f*depth  (R^2 %.3f)\n", brush, model->width_at_touch_mm,
                model->width_per_depth, r_squared);
    }
    img_free(&photo);
}

void app_run(void)
{
    hal_init();
    hal_log("leogoya planner\n");

    if (!hal_camera_init())
    {
        fatal("camera init");
    }
    if (!motion_init())
    {
        fatal("motion controller not responding / homing failed");
    }

    static brush_set_t brushes;
    brush_defaults(&brushes);
    if (hal_calibrate_requested())
    {
        calibrate_brushes(&brushes);
    }

    static planner_t planner;
    palette_t palette = { 0 };
    palette.count = PALETTE_N;
    if (!planner_init(&planner, &palette, &brushes, hal_millis() | 1))
    {
        fatal("oom (planner)");
    }

    hal_wait_button("Place the reference print (4 magenta corner dots) under the camera, press button");
    if (!capture(CAPTURE_REFERENCE, &planner.reference))
    {
        fatal("could not find the 4 markers on the reference");
    }
    palette_kmeans(&planner.reference, PALETTE_N, planner.rng.state, &planner.palette);
    palette_print(&planner.palette);
    hal_wait_button("Remove the reference, mix + fill the pots listed above, load brush 0, press button");
    dump("reference", &planner.reference);

    stroke_t *strokes = malloc(sizeof(stroke_t) * MAX_STROKES_PER_LAYER);
    lab_image_t photo;
    if (!strokes || !img_alloc(&photo, WORK_RES, WORK_RES))
    {
        fatal("oom (strokes)");
    }

    for (int layer = 0; layer < NUM_LAYERS; layer++)
    {
        /* closed loop: plan from what is really on the canvas */
        if (CLOSED_LOOP && capture(CAPTURE_CANVAS, &photo))
        {
            planner_set_canvas(&planner, &photo);
            hal_log("layer %d: planning from canvas photo (err %.1f)\n", layer, planner_error(&planner));
        }
        if (layer > 0 && k_layer_brush_id[layer] != k_layer_brush_id[layer - 1])
        {
            motion_rinse();
            motion_park();
            planner_brush_cleaned(&planner);
            char prompt[64];
            snprintf(prompt, sizeof prompt, "Swap to brush %d, press button", k_layer_brush_id[layer]);
            hal_wait_button(prompt);
        }

        layer_stats_t stats;
        uint32_t plan_start_ms = hal_millis();
        int stroke_count = planner_layer(&planner, layer, strokes, MAX_STROKES_PER_LAYER, &stats);
        hal_log("layer %d: %d/%d strokes accepted, mean dE %.1f -> %.1f, planned in %u ms\n", layer,
                stats.accepted_count, stats.candidate_count, stats.error_before, stats.error_after,
                (unsigned)(hal_millis() - plan_start_ms));
        char preview_name[24];
        snprintf(preview_name, sizeof preview_name, "sim_layer%d", layer);
        dump(preview_name, &planner.canvas_sim);

        for (int i = 0; i < stroke_count; i++)
        {
            const stroke_t *stroke = &strokes[i];
            if (stroke->reload != RELOAD_NONE &&
                !motion_load_paint(stroke->color, stroke->reload == RELOAD_RINSE))
            {
                fatal("paint reload");
            }
            if (!motion_paint_stroke(stroke, &brushes.brush[stroke->brush]))
            {
                fatal("stroke");
            }
        }
    }

    motion_rinse();
    motion_park();
    hal_log("done: %.1f m of brush strokes\n", motion_paint_distance() / 1000.0f);
    img_free(&photo);
    free(strokes);
    planner_free(&planner);
}
