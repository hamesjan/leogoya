#include "vision/palette.h"
#include "config.h"
#include "hal/hal.h"
#include "common/util.h"
#include <stdlib.h>
#include <float.h>
#include <string.h>

#define MAX_PALETTE_COLORS 16
#define UNASSIGNED         0xFF

int palette_nearest(const palette_t *palette, lab_t color)
{
    int nearest = 0;
    float nearest_dist = FLT_MAX;
    for (int i = 0; i < palette->count; i++)
    {
        float dist = lab_dist(palette->colors[i], color);
        if (dist < nearest_dist)
        {
            nearest_dist = dist;
            nearest = i;
        }
    }
    return nearest;
}

/* k-means++ seeding: each new center is a sample picked with probability
 * proportional to its squared distance from the nearest existing center. */
static void seed_centers(const lab_t *samples, int sample_count, float *dist_sq, rng_t *rng,
                         palette_t *palette)
{
    palette->colors[0] = samples[rng_range(rng, sample_count)];
    for (int center = 1; center < palette->count; center++)
    {
        double total_dist_sq = 0;
        for (int i = 0; i < sample_count; i++)
        {
            float nearest_dist = FLT_MAX;
            for (int existing = 0; existing < center; existing++)
            {
                float dist = lab_dist(samples[i], palette->colors[existing]);
                if (dist < nearest_dist)
                {
                    nearest_dist = dist;
                }
            }
            dist_sq[i] = nearest_dist * nearest_dist;
            total_dist_sq += dist_sq[i];
        }
        double target = rng_unit(rng) * total_dist_sq;
        double running_total = 0;
        int pick = sample_count - 1;
        for (int i = 0; i < sample_count; i++)
        {
            running_total += dist_sq[i];
            if (running_total >= target)
            {
                pick = i;
                break;
            }
        }
        palette->colors[center] = samples[pick];
    }
}

/* Lloyd iterations: assign every sample to its nearest center, move each center to its mean. */
static void refine_centers(const lab_t *samples, int sample_count, uint8_t *assignment, rng_t *rng,
                           palette_t *palette)
{
    for (int iteration = 0; iteration < KMEANS_ITERS; iteration++)
    {
        double sum_L[MAX_PALETTE_COLORS] = { 0 };
        double sum_a[MAX_PALETTE_COLORS] = { 0 };
        double sum_b[MAX_PALETTE_COLORS] = { 0 };
        int members[MAX_PALETTE_COLORS] = { 0 };
        int changed = 0;
        for (int i = 0; i < sample_count; i++)
        {
            int center = palette_nearest(palette, samples[i]);
            if (center != assignment[i])
            {
                changed++;
            }
            assignment[i] = (uint8_t)center;
            sum_L[center] += samples[i].L;
            sum_a[center] += samples[i].a;
            sum_b[center] += samples[i].b;
            members[center]++;
        }
        for (int center = 0; center < palette->count; center++)
        {
            if (members[center])
            {
                palette->colors[center] = (lab_t){ (float)(sum_L[center] / members[center]),
                                                   (float)(sum_a[center] / members[center]),
                                                   (float)(sum_b[center] / members[center]) };
            }
            else
            {
                palette->colors[center] = samples[rng_range(rng, sample_count)]; /* revive empty cluster */
            }
            palette->share[center] = (float)members[center] / sample_count;
        }
        if (iteration > 2 && changed == 0)
        {
            break;
        }
    }
}

/* Sort dark -> light: pots are laid out in that order, and painting dark
 * colors first within a layer lets lighter highlights land on top. */
static void sort_dark_to_light(palette_t *palette)
{
    for (int i = 0; i < palette->count; i++)
    {
        for (int j = i + 1; j < palette->count; j++)
        {
            if (palette->colors[j].L < palette->colors[i].L)
            {
                lab_t color = palette->colors[i];
                palette->colors[i] = palette->colors[j];
                palette->colors[j] = color;
                float share = palette->share[i];
                palette->share[i] = palette->share[j];
                palette->share[j] = share;
            }
        }
    }
}

void palette_kmeans(const lab_image_t *reference, int num_colors, uint32_t seed, palette_t *palette)
{
    lab_t *samples = malloc(sizeof(lab_t) * KMEANS_SAMPLES);
    uint8_t *assignment = malloc(KMEANS_SAMPLES);
    float *dist_sq = malloc(sizeof(float) * KMEANS_SAMPLES);
    if (!samples || !assignment || !dist_sq)
    {
        hal_log("kmeans: out of memory\n");
        palette->count = 0;
        free(samples);
        free(assignment);
        free(dist_sq);
        return;
    }
    memset(assignment, UNASSIGNED, KMEANS_SAMPLES);
    rng_t rng = { seed };
    int pixel_count = reference->width * reference->height;
    int sample_count = KMEANS_SAMPLES < pixel_count ? KMEANS_SAMPLES : pixel_count;
    if (num_colors > MAX_PALETTE_COLORS)
    {
        num_colors = MAX_PALETTE_COLORS;
    }

    for (int i = 0; i < sample_count; i++)
    {
        int pixel = rng_range(&rng, pixel_count);
        samples[i] = (lab_t){ reference->L[pixel], reference->a[pixel], reference->b[pixel] };
    }

    palette->count = num_colors;
    seed_centers(samples, sample_count, dist_sq, &rng, palette);
    refine_centers(samples, sample_count, assignment, &rng, palette);
    sort_dark_to_light(palette);

    free(samples);
    free(assignment);
    free(dist_sq);
}

static const char *mix_hint(lab_t color)
{
    if (color.L > 85)
    {
        return "mostly titanium white";
    }
    if (color.L < 20)
    {
        return "mars black / burnt umber";
    }
    float chroma_sq = color.a * color.a + color.b * color.b;
    if (chroma_sq < 15 * 15)
    {
        return color.L > 50 ? "white + touch of black (warm grey)" : "black + white (dark grey)";
    }
    if (color.a > 20 && color.b > 20)
    {
        return "cadmium red/orange + yellow";
    }
    if (color.a > 20)
    {
        return color.b < -10 ? "magenta/quinacridone + blue" : "cadmium/naphthol red";
    }
    if (color.b > 30)
    {
        return color.a < -10 ? "yellow + phthalo green" : "cadmium yellow / yellow ochre";
    }
    if (color.b < -20)
    {
        return "ultramarine / phthalo blue + white";
    }
    if (color.a < -15)
    {
        return "phthalo green + yellow";
    }
    return "earth tone: burnt sienna / raw umber + white";
}

void palette_print(const palette_t *palette)
{
    hal_log("=== Mix these %d pots (pot 0 is nearest the canvas) ===\n", palette->count);
    for (int i = 0; i < palette->count; i++)
    {
        lab_t color = palette->colors[i];
        uint8_t red;
        uint8_t green;
        uint8_t blue;
        lab_to_rgb(color, &red, &green, &blue);
        hal_log("  pot %d  #%02X%02X%02X  L=%5.1f a=%6.1f b=%6.1f  %4.1f%% of image  ~ %s\n", i, red, green,
                blue, color.L, color.a, color.b, 100.0f * palette->share[i], mix_hint(color));
    }
}
