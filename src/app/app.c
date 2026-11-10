#include "app/app.h"
#include "app/bootloader.h"
#include "planning/brush_model.h"
#include "config.h"
#include "hal/hal.h"
#include "motion/motion.h"
#include "vision/palette.h"
#include "planning/planner.h"
#include "vision/vision.h"
#include <stdio.h>
#include <string.h>

#define PALETTE_MAGIC  0x9A1E77E1u
#define PROGRESS_MAGIC 0x960E5501u
#define REF_MAGIC      0x2EF00001u

typedef struct { uint32_t magic; palette_t pal; } palette_rec_t;
typedef struct { uint32_t magic; uint32_t seed; int32_t next_layer; } progress_rec_t;

static const int k_layer_brush[NUM_LAYERS] = LAYER_BRUSH_ID;

static void fatal(const char *why) {
    hal_log("FATAL: %s\n", why);
    motion_park();
    hal_halt();
}

static void dump(const char *name, const lab_image_t *im) {
    uint8_t *rgb = hal_alloc_large((size_t)im->w * im->h * 3);
    if (!rgb) return;
    img_to_rgb8(im, rgb);
    hal_debug_dump_rgb(name, im->w, im->h, rgb);
    hal_free_large(rgb);
}

/* Photograph and rectify. Brush is parked first so the gantry is out of frame. */
static bool capture(capture_purpose_t why, lab_image_t *dst) {
    if (why == CAPTURE_CANVAS && !motion_park()) return false;
    hal_delay_ms(500);   /* let the gantry stop shaking */
    cam_frame_t f;
    if (!hal_camera_capture(why, &f)) return false;
    bool ok = vision_rectify(&f, dst);
    hal_camera_release(&f);
    return ok;
}

/* ---- reference: the one picture we keep ---- */
static bool save_reference(const lab_image_t *ref) {
    size_t n = (size_t)ref->w * ref->h * 3;
    uint8_t *buf = hal_alloc_large(n + 4);
    if (!buf) return false;
    uint32_t m = REF_MAGIC;
    memcpy(buf, &m, 4);
    img_to_rgb8(ref, buf + 4);
    bool ok = hal_storage_save(SLOT_REFERENCE, buf, n + 4);
    hal_free_large(buf);
    return ok;
}

static bool load_reference(lab_image_t *ref) {
    size_t n = (size_t)ref->w * ref->h * 3;
    uint8_t *buf = hal_alloc_large(n + 4);
    if (!buf) return false;
    uint32_t m = 0;
    bool ok = hal_storage_load(SLOT_REFERENCE, buf, n + 4);
    if (ok) memcpy(&m, buf, 4);
    ok = ok && m == REF_MAGIC;
    if (ok) img_from_rgb8(ref, buf + 4);
    hal_free_large(buf);
    return ok;
}

/* ---- brush calibration: the on-device regression ---- */
static void calibrate_brushes(brush_set_t *bs) {
    enum { LINES = 6 };
    float depth[NUM_BRUSHES][LINES];
    hal_wait_button("CAL: lay a scrap canvas with magenta corner dots on the bed, "
                    "fill pot 0 with dark paint, load brush 0, press button");
    for (int b = 0; b < NUM_BRUSHES; b++) {
        if (b > 0) {
            motion_rinse(); motion_park();
            char msg[64];
            snprintf(msg, sizeof msg, "CAL: swap to brush %d, press button", b);
            hal_wait_button(msg);
        }
        for (int i = 0; i < LINES; i++) {
            if (i % 2 == 0) motion_load_paint(0, false);
            depth[b][i] = (i + 0.5f) * Z_PRESS_MAX_MM / LINES;
            float y = 100.0f + (b * LINES + i) * 45.0f;
            motion_paint_line(100.0f, y, 400.0f, y, depth[b][i], &bs->b[b]);
        }
    }
    motion_rinse();

    lab_image_t img;
    if (!img_alloc(&img, WORK_RES, WORK_RES)) fatal("oom (cal)");
    if (!capture(CAPTURE_CANVAS, &img)) {
        hal_log("CAL: no canvas photo - keeping previous brush model\n");
        img_free(&img);
        return;
    }
    dump("calibration", &img);
    lab_t bg = img_get(&img, (int)(700 / MM_PER_PX), (int)(100 / MM_PER_PX)); /* unpainted area */
    for (int b = 0; b < NUM_BRUSHES; b++) {
        float w[LINES];
        for (int i = 0; i < LINES; i++) {
            int row = (int)((100.0f + (b * LINES + i) * 45.0f) / MM_PER_PX);
            w[i] = brush_measure_width(&img, bg, row, (int)(150 / MM_PER_PX), (int)(350 / MM_PER_PX));
            hal_log("CAL: brush %d depth %.2f mm -> width %.1f mm\n", b, depth[b][i], w[i]);
        }
        float r2 = brush_fit(&bs->b[b], depth[b], w, LINES);
        hal_log("CAL: brush %d  width = %.2f + %.2f*depth  (R^2 %.3f)\n", b, bs->b[b].w0, bs->b[b].k, r2);
    }
    img_free(&img);
    hal_storage_save(SLOT_BRUSH, bs, sizeof *bs);
}

void app_run(void) {
    hal_init();
    boot_mode_t mode = boot_init();
    hal_log("leogoya: boot mode %s\n", boot_mode_name(mode));
    if (mode == BOOT_SAFE) {
        hal_wait_button("SAFE MODE: check logs, press button to clear and reboot into normal mode");
        hal_boot_counter_set(0);
        fatal("reboot required");
    }

    if (!hal_camera_init()) fatal("camera init");
    if (!motion_init()) fatal("motion board not responding / homing failed");
    boot_mark_healthy();

    static brush_set_t brushes;
    if (!hal_storage_load(SLOT_BRUSH, &brushes, sizeof brushes) || brushes.magic != BRUSH_MAGIC)
        brush_defaults(&brushes);
    if (mode == BOOT_CALIBRATE) calibrate_brushes(&brushes);

    /* resume? */
    progress_rec_t prog;
    palette_rec_t palrec;
    bool resume = hal_storage_load(SLOT_PROGRESS, &prog, sizeof prog) && prog.magic == PROGRESS_MAGIC &&
                  prog.next_layer > 0 && prog.next_layer < NUM_LAYERS &&
                  hal_storage_load(SLOT_PALETTE, &palrec, sizeof palrec) && palrec.magic == PALETTE_MAGIC;

    static planner_t P;
    palette_t pal = { 0 };
    pal.n = PALETTE_N;
    if (!planner_init(&P, &pal, &brushes, resume ? prog.seed : hal_millis() | 1)) fatal("oom (planner)");

    if (resume && load_reference(&P.ref)) {
        P.pal = palrec.pal;
        hal_log("resuming at layer %d\n", (int)prog.next_layer);
    } else {
        resume = false;
        hal_wait_button("Place the reference print (4 magenta corner dots) under the camera, press button");
        if (!capture(CAPTURE_REFERENCE, &P.ref)) fatal("could not find the 4 markers on the reference");
        save_reference(&P.ref);

        palette_kmeans(&P.ref, PALETTE_N, P.rng.s, &P.pal);
        palette_print(&P.pal);
        palrec.magic = PALETTE_MAGIC; palrec.pal = P.pal;
        hal_storage_save(SLOT_PALETTE, &palrec, sizeof palrec);
        prog = (progress_rec_t){ PROGRESS_MAGIC, P.rng.s, 0 };
        hal_storage_save(SLOT_PROGRESS, &prog, sizeof prog);
        hal_wait_button("Remove the reference, mix + fill the pots listed above, load brush 0, press button");
    }
    dump("reference", &P.ref);

    stroke_t *strokes = hal_alloc_large(sizeof(stroke_t) * MAX_STROKES_PER_LAYER);
    lab_image_t photo;
    if (!strokes || !img_alloc(&photo, WORK_RES, WORK_RES)) fatal("oom (strokes)");

    for (int layer = resume ? prog.next_layer : 0; layer < NUM_LAYERS; layer++) {
        /* closed loop: plan from what is really on the canvas */
        if (CLOSED_LOOP && capture(CAPTURE_CANVAS, &photo)) {
            planner_set_canvas(&P, &photo);
            hal_log("layer %d: planning from canvas photo (err %.1f)\n", layer, planner_error(&P));
        }
        if (layer > 0 && k_layer_brush[layer] != k_layer_brush[layer - 1]) {
            motion_rinse(); motion_park();
            planner_brush_cleaned(&P);
            char msg[64];
            snprintf(msg, sizeof msg, "Swap to brush %d, press button", k_layer_brush[layer]);
            hal_wait_button(msg);
        }

        layer_stats_t st;
        uint32_t t0 = hal_millis();
        int n = planner_layer(&P, layer, strokes, MAX_STROKES_PER_LAYER, &st);
        hal_log("layer %d: %d/%d strokes accepted, mean dE %.1f -> %.1f, planned in %u ms\n",
                layer, st.accepted, st.candidates, st.err_before, st.err_after,
                (unsigned)(hal_millis() - t0));
        char name[24];
        snprintf(name, sizeof name, "sim_layer%d", layer);
        dump(name, &P.sim);

        for (int i = 0; i < n; i++) {
            const stroke_t *s = &strokes[i];
            if (s->reload && !motion_load_paint(s->color, s->reload == 2)) fatal("paint reload");
            if (!motion_paint_stroke(s, &brushes.b[s->brush])) fatal("stroke");
            hal_led(i & 1);
        }
        prog.next_layer = layer + 1;
        hal_storage_save(SLOT_PROGRESS, &prog, sizeof prog);
    }

    motion_rinse();
    motion_park();
    prog.next_layer = NUM_LAYERS;      /* finished: next boot starts a new painting */
    prog.magic = 0;
    hal_storage_save(SLOT_PROGRESS, &prog, sizeof prog);
    hal_log("done: %.1f m of brush strokes\n", motion_paint_distance() / 1000.0f);
    img_free(&photo);
    hal_free_large(strokes);
    planner_free(&P);
    hal_led(true);
}
