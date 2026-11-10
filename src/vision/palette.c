#include "vision/palette.h"
#include "config.h"
#include "hal/hal.h"
#include "common/util.h"
#include <float.h>
#include <string.h>

int palette_nearest(const palette_t *p, lab_t c) {
    int best = 0;
    float bd = FLT_MAX;
    for (int i = 0; i < p->n; i++) {
        float d = lab_dist(p->c[i], c);
        if (d < bd) { bd = d; best = i; }
    }
    return best;
}

void palette_kmeans(const lab_image_t *ref, int k, uint32_t seed, palette_t *out) {
    /* scratch lives in PSRAM: internal SRAM is reserved for camera DMA */
    lab_t *samples = hal_alloc_large(sizeof(lab_t) * KMEANS_SAMPLES);
    uint8_t *assign = hal_alloc_large(KMEANS_SAMPLES);
    float *d2 = hal_alloc_large(sizeof(float) * KMEANS_SAMPLES);
    if (!samples || !assign || !d2) {
        hal_log("kmeans: out of memory\n");
        out->n = 0;
        goto done;
    }
    memset(assign, 0xFF, KMEANS_SAMPLES);
    rng_t rng = { seed };
    int npx = ref->w * ref->h;
    int ns = KMEANS_SAMPLES < npx ? KMEANS_SAMPLES : npx;
    if (k > 16) k = 16;

    for (int i = 0; i < ns; i++) {
        int j = rng_range(&rng, npx);
        samples[i] = (lab_t){ ref->L[j], ref->a[j], ref->b[j] };
    }

    /* k-means++ seeding: spread initial centers proportional to D^2 */
    out->n = k;
    out->c[0] = samples[rng_range(&rng, ns)];
    for (int c = 1; c < k; c++) {
        double total = 0;
        for (int i = 0; i < ns; i++) {
            float bd = FLT_MAX;
            for (int j = 0; j < c; j++) {
                float d = lab_dist(samples[i], out->c[j]);
                if (d < bd) bd = d;
            }
            d2[i] = bd * bd; total += d2[i];
        }
        double t = rng_unit(&rng) * total, acc = 0;
        int pick = ns - 1;
        for (int i = 0; i < ns; i++) { acc += d2[i]; if (acc >= t) { pick = i; break; } }
        out->c[c] = samples[pick];
    }

    /* Lloyd iterations */
    for (int it = 0; it < KMEANS_ITERS; it++) {
        double sL[16] = {0}, sa[16] = {0}, sb[16] = {0};
        int cnt[16] = {0}, changed = 0;
        for (int i = 0; i < ns; i++) {
            int a = palette_nearest(out, samples[i]);
            if (a != assign[i]) changed++;
            assign[i] = (uint8_t)a;
            sL[a] += samples[i].L; sa[a] += samples[i].a; sb[a] += samples[i].b; cnt[a]++;
        }
        for (int c = 0; c < k; c++) {
            if (cnt[c]) out->c[c] = (lab_t){ (float)(sL[c] / cnt[c]), (float)(sa[c] / cnt[c]), (float)(sb[c] / cnt[c]) };
            else out->c[c] = samples[rng_range(&rng, ns)];  /* revive empty cluster */
            out->share[c] = (float)cnt[c] / ns;
        }
        if (it > 2 && changed == 0) break;
    }

    /* sort dark -> light: pots are laid out in that order, and painting dark
     * colors first within a layer lets lighter highlights land on top */
    for (int i = 0; i < k; i++)
        for (int j = i + 1; j < k; j++)
            if (out->c[j].L < out->c[i].L) {
                lab_t t = out->c[i]; out->c[i] = out->c[j]; out->c[j] = t;
                float s = out->share[i]; out->share[i] = out->share[j]; out->share[j] = s;
            }
done:
    hal_free_large(samples);
    hal_free_large(assign);
    hal_free_large(d2);
}

void palette_measure(const lab_image_t *canvas, palette_t *p) {
    int y = canvas->h / 2, half = canvas->w / (p->n * 2) / 3;
    for (int i = 0; i < p->n; i++) {
        int cx = (int)((i + 0.5f) * canvas->w / p->n);
        double L = 0, a = 0, b = 0; int n = 0;
        for (int dy = -half; dy <= half; dy++)
            for (int dx = -half; dx <= half; dx++) {
                lab_t c = img_get(canvas, cx + dx, y + dy);
                L += c.L; a += c.a; b += c.b; n++;
            }
        p->c[i] = (lab_t){ (float)(L / n), (float)(a / n), (float)(b / n) };
    }
}

static const char *mix_hint(lab_t c) {
    if (c.L > 85) return "mostly titanium white";
    if (c.L < 20) return "mars black / burnt umber";
    float chroma = c.a * c.a + c.b * c.b;
    if (chroma < 15 * 15) return c.L > 50 ? "white + touch of black (warm grey)" : "black + white (dark grey)";
    if (c.a > 20 && c.b > 20) return "cadmium red/orange + yellow";
    if (c.a > 20) return c.b < -10 ? "magenta/quinacridone + blue" : "cadmium/naphthol red";
    if (c.b > 30) return c.a < -10 ? "yellow + phthalo green" : "cadmium yellow / yellow ochre";
    if (c.b < -20) return "ultramarine / phthalo blue + white";
    if (c.a < -15) return "phthalo green + yellow";
    return "earth tone: burnt sienna / raw umber + white";
}

void palette_print(const palette_t *p) {
    hal_log("=== Mix these %d pots (pot 0 is nearest the canvas) ===\n", p->n);
    for (int i = 0; i < p->n; i++) {
        uint8_t r, g, b;
        lab_to_rgb(p->c[i], &r, &g, &b);
        hal_log("  pot %d  #%02X%02X%02X  L=%5.1f a=%6.1f b=%6.1f  %4.1f%% of image  ~ %s\n",
                i, r, g, b, p->c[i].L, p->c[i].a, p->c[i].b, 100.0f * p->share[i], mix_hint(p->c[i]));
    }
}
