#include "vision/vision.h"
#include "config.h"
#include <math.h>
#include <string.h>

enum
{
    CORNER_TOP_LEFT,
    CORNER_TOP_RIGHT,
    CORNER_BOTTOM_RIGHT,
    CORNER_BOTTOM_LEFT,
    NUM_CORNERS
};

typedef struct
{
    int red;
    int green;
    int blue;
} rgb_t;

static inline rgb_t rgb565_unpack(uint16_t pixel)
{
    rgb_t color;
    color.red = ((pixel >> 11) & 31) * 255 / 31;
    color.green = ((pixel >> 5) & 63) * 255 / 63;
    color.blue = (pixel & 31) * 255 / 31;
    return color;
}

/* Strong magenta: red and blue high, green well below both. */
static inline bool is_marker(rgb_t color)
{
    int weaker_of_red_blue = color.red < color.blue ? color.red : color.blue;
    return color.red > 140 && color.blue > 110 && color.green * 10 < weaker_of_red_blue * 6;
}

bool vision_find_markers(const cam_frame_t *frame, point2_t corners[4])
{
    double sum_x[NUM_CORNERS] = { 0 };
    double sum_y[NUM_CORNERS] = { 0 };
    int pixel_count[NUM_CORNERS] = { 0 };
    int half_width = frame->width / 2;
    int half_height = frame->height / 2;
    for (int y = 0; y < frame->height; y++)
    {
        for (int x = 0; x < frame->width; x++)
        {
            if (!is_marker(rgb565_unpack(frame->rgb565[y * frame->width + x])))
            {
                continue;
            }
            int corner;
            if (y < half_height)
            {
                corner = x < half_width ? CORNER_TOP_LEFT : CORNER_TOP_RIGHT;
            }
            else
            {
                corner = x < half_width ? CORNER_BOTTOM_LEFT : CORNER_BOTTOM_RIGHT;
            }
            sum_x[corner] += x;
            sum_y[corner] += y;
            pixel_count[corner]++;
        }
    }
    for (int corner = 0; corner < NUM_CORNERS; corner++)
    {
        if (pixel_count[corner] < MARKER_MIN_PIXELS)
        {
            hal_log("vision: marker %d not found (%d px)\n", corner, pixel_count[corner]);
            return false;
        }
        corners[corner].x = (float)(sum_x[corner] / pixel_count[corner]);
        corners[corner].y = (float)(sum_y[corner] / pixel_count[corner]);
    }
    return true;
}

/* Solve the 8x8 linear system in augmented form [matrix | rhs] by Gauss-Jordan
 * elimination with partial pivoting; the solution ends up in column 8. */
static bool solve_8x8(double system[8][9])
{
    for (int col = 0; col < 8; col++)
    {
        int pivot_row = col;
        for (int row = col + 1; row < 8; row++)
        {
            if (fabs(system[row][col]) > fabs(system[pivot_row][col]))
            {
                pivot_row = row;
            }
        }
        if (fabs(system[pivot_row][col]) < 1e-12)
        {
            return false; /* singular: markers are collinear or coincident */
        }
        if (pivot_row != col)
        {
            for (int k = 0; k < 9; k++)
            {
                double swap = system[col][k];
                system[col][k] = system[pivot_row][k];
                system[pivot_row][k] = swap;
            }
        }
        for (int row = 0; row < 8; row++)
        {
            if (row == col)
            {
                continue;
            }
            double factor = system[row][col] / system[col][col];
            for (int k = col; k < 9; k++)
            {
                system[row][k] -= factor * system[col][k];
            }
        }
    }
    for (int row = 0; row < 8; row++)
    {
        system[row][8] /= system[row][row];
    }
    return true;
}

bool vision_homography(const point2_t to_points[4], const point2_t from_points[4], float homography[9])
{
    /* Each correspondence (u,v) -> (x,y) gives two linear equations in h0..h7:
     *   x = (h0 u + h1 v + h2) / (h6 u + h7 v + 1)
     *   y = (h3 u + h4 v + h5) / (h6 u + h7 v + 1) */
    double system[8][9];
    for (int i = 0; i < 4; i++)
    {
        double u = to_points[i].x;
        double v = to_points[i].y;
        double x = from_points[i].x;
        double y = from_points[i].y;
        double x_equation[9] = { u, v, 1, 0, 0, 0, -u * x, -v * x, x };
        double y_equation[9] = { 0, 0, 0, u, v, 1, -u * y, -v * y, y };
        memcpy(system[2 * i], x_equation, sizeof x_equation);
        memcpy(system[2 * i + 1], y_equation, sizeof y_equation);
    }
    if (!solve_8x8(system))
    {
        return false;
    }
    for (int i = 0; i < 8; i++)
    {
        homography[i] = (float)system[i][8];
    }
    homography[8] = 1.0f;
    return true;
}

bool vision_rectify(const cam_frame_t *frame, lab_image_t *rectified)
{
    point2_t marker_corners[NUM_CORNERS];
    if (!vision_find_markers(frame, marker_corners))
    {
        return false;
    }
    float out_width = (float)rectified->width;
    float out_height = (float)rectified->height;
    point2_t image_corners[NUM_CORNERS] = {
        { 0, 0 }, { out_width, 0 }, { out_width, out_height }, { 0, out_height }
    };
    float homography[9];
    if (!vision_homography(image_corners, marker_corners, homography))
    {
        return false;
    }
    const float *h = homography;

    /* Average a 2x2 grid of camera samples per output pixel (cheap anti-aliasing). */
    for (int v = 0; v < rectified->height; v++)
    {
        for (int u = 0; u < rectified->width; u++)
        {
            int sum_red = 0;
            int sum_green = 0;
            int sum_blue = 0;
            int sample_count = 0;
            for (int sub_y = 0; sub_y < 2; sub_y++)
            {
                for (int sub_x = 0; sub_x < 2; sub_x++)
                {
                    float sample_u = u + 0.25f + 0.5f * sub_x;
                    float sample_v = v + 0.25f + 0.5f * sub_y;
                    float denominator = h[6] * sample_u + h[7] * sample_v + 1.0f;
                    int camera_x = (int)((h[0] * sample_u + h[1] * sample_v + h[2]) / denominator);
                    int camera_y = (int)((h[3] * sample_u + h[4] * sample_v + h[5]) / denominator);
                    if (camera_x < 0 || camera_y < 0 || camera_x >= frame->width || camera_y >= frame->height)
                    {
                        continue;
                    }
                    rgb_t color = rgb565_unpack(frame->rgb565[camera_y * frame->width + camera_x]);
                    sum_red += color.red;
                    sum_green += color.green;
                    sum_blue += color.blue;
                    sample_count++;
                }
            }
            if (!sample_count)
            {
                sample_count = 1;
            }
            lab_t lab = rgb_to_lab((uint8_t)(sum_red / sample_count), (uint8_t)(sum_green / sample_count),
                                   (uint8_t)(sum_blue / sample_count));
            img_set(rectified, u, v, lab);
        }
    }
    return true;
}
