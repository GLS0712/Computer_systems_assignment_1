// til at compilere (win): gcc cbmp.c main.c -o main.exe -std=c99
// til at køre (win): main.exe example.bmp [output.bmp]
// skal køres i din computers terminal, ikke vs-code-terminalen
//
// Celledetektion med erosion:
//   1. Indlæs billede
//   2. Konverter til gråtone
//   3. Binær threshold (Otsu's metode)
//   4. Erodér gentagne gange (skiftevis kryds og kvadrat). Efter hver
//      omgang, kig efter celler der er skrumpet til en lille isoleret plet.
//   5. Tegn en markør på hver celle
//   6. Gem outputbillede og udskriv resultater

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "cbmp.h"
#include "time.h"

// Creating a directory is one of the few things that isn't the same call
// on Windows vs Linux/Mac, so we pick the right one at compile time.
// _WIN32 is defined automatically by the compiler when building on Windows.
#ifdef _WIN32
#include <direct.h>
#define MKDIR(path) _mkdir(path)
#else
#include <sys/stat.h>
#include <sys/types.h>
#define MKDIR(path) mkdir(path, 0755)
#endif

/* ------------------------------------------------------------------ */
/* Parametre                                                            */
/* ------------------------------------------------------------------ */

#define MAX_CELLS 2000

#define CAPTURE_SIZE 12       // capture-området er CAPTURE_SIZE x CAPTURE_SIZE
#define EXCLUSION_FRAME 1     // bredden af den sorte ring omkring capture-området
#define MIN_CELL_DISTANCE 11  // detektioner tættere end dette (px) er samme celle

#define SAVE_STEP_IMAGES 1    // 1 = gem et billede efter hver erosions-omgang

#define MARKER_ARM_LENGHT 5
#define MARKER_R 255
#define MARKER_G 0
#define MARKER_B 0

/* ------------------------------------------------------------------ */
/* Små hjælpefunktioner                                                 */
/* ------------------------------------------------------------------ */

static int inside_image(int x, int y)
{
    return x >= 0 && x < BMP_WIDTH && y >= 0 && y < BMP_HEIGTH;
}

// Creates a directory if it doesn't already exist. Safe to call on a path
// that already exists, and with an empty string (= current directory).
static void ensure_directory(const char *path)
{
    if (path[0] == '\0')
        return;
    if (MKDIR(path) != 0 && errno != EEXIST)
        fprintf(stderr, "Warning: could not create directory '%s'\n", path);
}

// "samples/easy/1EASY.bmp" -> "results_example/1EASY_out.bmp"
static void get_default_output_path(const char *input_path, char *output_path, size_t output_path_size)
{
    const char *last_slash = strrchr(input_path, '/');
    const char *last_bslash = strrchr(input_path, '\\');
    const char *sep = last_slash;
    if (last_bslash && (!sep || last_bslash > sep))
        sep = last_bslash;

    const char *base = sep ? sep + 1 : input_path;

    char name[256];
    strncpy(name, base, sizeof(name) - 1);
    name[sizeof(name) - 1] = '\0';
    char *dot = strrchr(name, '.');
    if (dot)
        *dot = '\0';

    snprintf(output_path, output_path_size, "results_example/%s_out.bmp", name);
}

// From "results_example/1EASY_out.bmp" works out:
//   output_dir = "results_example"
//   steps_dir  = "results_example/1EASY_out_steps" (debug images for this image)
static void get_output_folders(const char *output_path,
                               char *output_dir, size_t output_dir_size,
                               char *steps_dir, size_t steps_dir_size)
{
    const char *last_slash = strrchr(output_path, '/');
    const char *last_bslash = strrchr(output_path, '\\');
    const char *sep = last_slash;
    if (last_bslash && (!sep || last_bslash > sep))
        sep = last_bslash;

    char base[256];
    if (sep)
    {
        size_t dir_len = (size_t)(sep - output_path);
        if (dir_len >= output_dir_size)
            dir_len = output_dir_size - 1;
        memcpy(output_dir, output_path, dir_len);
        output_dir[dir_len] = '\0';
        strncpy(base, sep + 1, sizeof(base) - 1);
        base[sizeof(base) - 1] = '\0';
    }
    else
    {
        output_dir[0] = '\0';
        strncpy(base, output_path, sizeof(base) - 1);
        base[sizeof(base) - 1] = '\0';
    }

    // Fjern filendelsen, f.eks. "1EASY_out.bmp" -> "1EASY_out".
    char *dot = strrchr(base, '.');
    if (dot)
        *dot = '\0';

    if (output_dir[0] != '\0')
        snprintf(steps_dir, steps_dir_size, "%s/%s_steps", output_dir, base);
    else
        snprintf(steps_dir, steps_dir_size, "%s_steps", base);
}

/* ------------------------------------------------------------------ */
/* Step 2: gråtone                                                      */
/* ------------------------------------------------------------------ */

void convert_to_grayscale(unsigned char image[BMP_WIDTH][BMP_HEIGTH][BMP_CHANNELS],
                          unsigned char gray[BMP_WIDTH][BMP_HEIGTH])
{
    for (int x = 0; x < BMP_WIDTH; x++)
        for (int y = 0; y < BMP_HEIGTH; y++)
        {
            // gennemsnit af rød, grøn og blå -> ét gråtone-tal pr. pixel
            int sum = image[x][y][0] + image[x][y][1] + image[x][y][2];
            gray[x][y] = (unsigned char)(sum / 3);
        }
}

/* ------------------------------------------------------------------ */
/* Step 3: threshold (Otsu's metode)                                    */
/* ------------------------------------------------------------------ */

// Prøver hver threshold t, som deler pixels op i baggrund (gray <= t)
// og forgrund (gray > t), og vælger den t der maksimerer
//   count0 * count1 * (mean0 - mean1)^2
// altså den opdeling hvor begge grupper er store OG meget forskellige i lysstyrke.
int find_threshold(unsigned char gray[BMP_WIDTH][BMP_HEIGTH])
{
    int histogram[256] = {0};
    for (int x = 0; x < BMP_WIDTH; x++)
        for (int y = 0; y < BMP_HEIGTH; y++)
            histogram[gray[x][y]]++;

    long total_count = (long)BMP_WIDTH * BMP_HEIGTH;
    long total_sum = 0;
    for (int i = 0; i < 256; i++)
        total_sum += (long)i * histogram[i];

    // Løbende totaler for baggrundssiden, så hver t er O(1).
    long count0 = 0, sum0 = 0;
    double best_variance = -1.0;
    int best_t = 0;

    for (int t = 0; t < 255; t++)
    {
        count0 += histogram[t];
        sum0 += (long)t * histogram[t];

        long count1 = total_count - count0;
        if (count0 == 0 || count1 == 0)
            continue; // alt på én side, ingen gyldig opdeling

        double mean0 = (double)sum0 / (double)count0;
        double mean1 = (double)(total_sum - sum0) / (double)count1;
        double diff = mean0 - mean1;
        double variance = (double)count0 * (double)count1 * diff * diff;

        if (variance > best_variance)
        {
            best_variance = variance;
            best_t = t;
        }
    }
    return best_t;
}

void apply_threshold(unsigned char gray[BMP_WIDTH][BMP_HEIGTH],
                     unsigned char binary[BMP_WIDTH][BMP_HEIGTH], int threshold)
{
    for (int x = 0; x < BMP_WIDTH; x++)
        for (int y = 0; y < BMP_HEIGTH; y++)
            // under/lig threshold bliver sort (0), resten bliver hvid (255)
            binary[x][y] = (gray[x][y] <= threshold) ? 0 : 255;
}

/* ------------------------------------------------------------------ */
/* Step 4a: erosion                                                    */
/* ------------------------------------------------------------------ */

// Én erosions-omgang fra `binary` til `eroded`. En hvid pixel overlever kun
// hvis dens nabopixels også er hvide:
//   use_square = 0 -> kryds   (op/ned/venstre/højre)
//   use_square = 1 -> kvadrat (også de 4 diagonaler)
// Ved at skifte mellem de to skrumper celler i en rund (ottekantet) form
// i stedet for en diamant, så uregelmæssige celler splitter i to mindre ofte.
// Returnerer 1 hvis en pixel blev fjernet, 0 hvis billedet ikke ændrede sig.
int erode_image(unsigned char binary[BMP_WIDTH][BMP_HEIGTH],
                unsigned char eroded[BMP_WIDTH][BMP_HEIGTH], int use_square)
{
    int changed = 0;
    for (int x = 0; x < BMP_WIDTH; x++)
        for (int y = 0; y < BMP_HEIGTH; y++)
        {
            if (binary[x][y] == 0)
            {
                eroded[x][y] = 0; // sort forbliver sort
                continue;
            }

            // pixels på billedets kant dør altid
            if (x == 0 || y == 0 || x == BMP_WIDTH - 1 || y == BMP_HEIGTH - 1)
            {
                eroded[x][y] = 0;
                changed = 1;
                continue;
            }

            int survives = binary[x - 1][y] == 255 && binary[x + 1][y] == 255 &&
                           binary[x][y - 1] == 255 && binary[x][y + 1] == 255;

            if (use_square)
                survives = survives &&
                           binary[x - 1][y - 1] == 255 && binary[x + 1][y - 1] == 255 &&
                           binary[x - 1][y + 1] == 255 && binary[x + 1][y + 1] == 255;

            if (survives)
            {
                eroded[x][y] = 255;
            }
            else
            {
                eroded[x][y] = 0; // pixel på kanten af et hvidt område bliver "spist"
                changed = 1;
            }
        }
    return changed;
}

/* ------------------------------------------------------------------ */
/* Step 4b: pletdetektion                                               */
/* ------------------------------------------------------------------ */

// Findes der allerede en celle inden for MIN_CELL_DISTANCE af (x,y)?
// Fanger celler som erosionen har splittet i to stykker.
static int is_near_existing(int coords[MAX_CELLS][2], int cell_count, int x, int y)
{
    for (int i = 0; i < cell_count; i++)
    {
        int dx = coords[i][0] - x;
        int dy = coords[i][1] - y;
        if (dx * dx + dy * dy < MIN_CELL_DISTANCE * MIN_CELL_DISTANCE)
            return 1;
    }
    return 0;
}

// Skyder et 12x12 capture-område over billedet. En celle detekteres når
//   1. capture-området indeholder mindst én hvid pixel, og
//   2. exclusion-ringen omkring det er helt sort.
// Pixels uden for billedet tæller som sorte, så celler ved kanten bliver fundet.
// Den detekterede celle fjernes derefter, så den ikke kan tælles igen.
// Returnerer antallet af nye celler fundet i denne omgang.
int detect_spots(unsigned char binary[BMP_WIDTH][BMP_HEIGTH],
                 int coords[MAX_CELLS][2], int *cell_count)
{
    int detections_found = 0;
    int half_before_center = CAPTURE_SIZE / 2;
    int half_after_center = CAPTURE_SIZE / 2 - 1; // giver præcis 12x12

    for (int pixel_x = 0; pixel_x < BMP_WIDTH; pixel_x++)
        for (int pixel_y = 0; pixel_y < BMP_HEIGTH; pixel_y++)
        {
            int capture_left = pixel_x - half_before_center;
            int capture_right = pixel_x + half_after_center;
            int capture_top = pixel_y - half_before_center;
            int capture_bottom = pixel_y + half_after_center;
            int window_left = capture_left - EXCLUSION_FRAME;
            int window_right = capture_right + EXCLUSION_FRAME;
            int window_top = capture_top - EXCLUSION_FRAME;
            int window_bottom = capture_bottom + EXCLUSION_FRAME;

            // Betingelse 1: mindst én hvid pixel inde i capture-området.
            int found_white_pixel = 0;
            for (int cx = capture_left; cx <= capture_right && !found_white_pixel; cx++)
                for (int cy = capture_top; cy <= capture_bottom; cy++)
                    if (inside_image(cx, cy) && binary[cx][cy] == 255)
                    {
                        found_white_pixel = 1;
                        break;
                    }
            if (!found_white_pixel)
                continue;

            // Betingelse 2: exclusion-ringen er helt sort.
            int exclusion_ring_is_black = 1;
            for (int wx = window_left; wx <= window_right && exclusion_ring_is_black; wx++)
                for (int wy = window_top; wy <= window_bottom; wy++)
                {
                    if (!inside_image(wx, wy))
                        continue; // uden for billedet tæller som sort
                    int inside_capture_area = wx >= capture_left && wx <= capture_right &&
                                              wy >= capture_top && wy <= capture_bottom;
                    if (!inside_capture_area && binary[wx][wy] == 255)
                    {
                        exclusion_ring_is_black = 0;
                        break;
                    }
                }
            if (!exclusion_ring_is_black)
                continue;

            // Find centrum af den hvide plet (gennemsnit af de hvide pixels)
            // og gør samtidig capture-området sort.
            long sum_x = 0, sum_y = 0, white_count = 0;
            for (int cx = capture_left; cx <= capture_right; cx++)
                for (int cy = capture_top; cy <= capture_bottom; cy++)
                {
                    if (!inside_image(cx, cy))
                        continue;
                    if (binary[cx][cy] == 255)
                    {
                        sum_x += cx;
                        sum_y += cy;
                        white_count++;
                    }
                    binary[cx][cy] = 0;
                }
            int center_x = (int)(sum_x / white_count); // white_count > 0, betingelse 1 garanterer det
            int center_y = (int)(sum_y / white_count);

            // Samme celle, som erosionen har splittet i to? Så tæl den ikke igen.
            if (is_near_existing(coords, *cell_count, center_x, center_y))
                continue;

            if (*cell_count < MAX_CELLS)
            {
                coords[*cell_count][0] = center_x;
                coords[*cell_count][1] = center_y;
                (*cell_count)++;
                detections_found++;
            }
            else
            {
                fprintf(stderr, "Warning: MAX_CELLS reached, dropping detection at (%d,%d)\n", center_x, center_y);
            }
        }
    return detections_found;
}

/* ------------------------------------------------------------------ */
/* Step 5: tegn markører                                                */
/* ------------------------------------------------------------------ */

void generate_output_image(unsigned char color_image[BMP_WIDTH][BMP_HEIGTH][BMP_CHANNELS],
                           int coords[MAX_CELLS][2], int cell_count)
{
    for (int i = 0; i < cell_count; i++)
    {
        int cx = coords[i][0];
        int cy = coords[i][1];

        // horisontal arm
        for (int dx = -MARKER_ARM_LENGHT; dx <= MARKER_ARM_LENGHT; dx++)
        {
            int x = cx + dx;
            if (x < 0 || x >= BMP_WIDTH)
                continue;
            color_image[x][cy][0] = MARKER_R;
            color_image[x][cy][1] = MARKER_G;
            color_image[x][cy][2] = MARKER_B;
        }

        // vertikal arm
        for (int dy = -MARKER_ARM_LENGHT; dy <= MARKER_ARM_LENGHT; dy++)
        {
            int y = cy + dy;
            if (y < 0 || y >= BMP_HEIGTH)
                continue;
            color_image[cx][y][0] = MARKER_R;
            color_image[cx][y][1] = MARKER_G;
            color_image[cx][y][2] = MARKER_B;
        }
    }
}

/* ------------------------------------------------------------------ */
/* main                                                                 */
/* ------------------------------------------------------------------ */

int main(int argc, char *argv[])
{
    if (argc != 2 && argc != 3)
    {
        fprintf(stderr, "Usage: %s <input.bmp> [output.bmp]\n", argv[0]);
        return 1;
    }
    char *input_path = argv[1];
    char default_output_path[300];
    char *output_path;
    if (argc == 3)
    {
        output_path = argv[2];
    }
    else
    {
        get_default_output_path(input_path, default_output_path, sizeof(default_output_path));
        output_path = default_output_path;
    }

    static unsigned char color_image[BMP_WIDTH][BMP_HEIGTH][BMP_CHANNELS];
    static unsigned char gray_image[BMP_WIDTH][BMP_HEIGTH];
    static unsigned char binary_image[BMP_WIDTH][BMP_HEIGTH];
    static unsigned char eroded_image[BMP_WIDTH][BMP_HEIGTH];
#if SAVE_STEP_IMAGES
    static unsigned char step_image[BMP_WIDTH][BMP_HEIGTH][BMP_CHANNELS];
#endif
    static int coords[MAX_CELLS][2];
    int cell_count = 0;

    char output_dir[256];
    char steps_dir[300];
    get_output_folders(output_path, output_dir, sizeof(output_dir),
                       steps_dir, sizeof(steps_dir));
    ensure_directory(output_dir);
#if SAVE_STEP_IMAGES
    ensure_directory(steps_dir);
    char step_filename[512];
#endif

    // Step 1
    read_bitmap(input_path, color_image);
    printf("Loaded '%s' (%d x %d, %d channels)\n",
           input_path, BMP_WIDTH, BMP_HEIGTH, BMP_CHANNELS);

    // Step 2
    convert_to_grayscale(color_image, gray_image);

    // Step 3
    int threshold = find_threshold(gray_image);
    printf("Dynamic threshold: %d\n", threshold);
    apply_threshold(gray_image, binary_image, threshold);

    // Step 4: erodér indtil billedet er helt sort.
    // "current" og "next" peger på de to buffere og bliver byttet om efter
    // hver omgang, så billedet aldrig kopieres (ping-pong buffering).
    unsigned char (*current)[BMP_HEIGTH] = binary_image;
    unsigned char (*next)[BMP_HEIGTH] = eroded_image;

    int changed = 1;
    int passes = 0;
    while (changed)
    {
        // passes % 2: kryds på lige omgange, kvadrat på ulige omgange
        changed = erode_image(current, next, passes % 2);
        passes++;

        int found = detect_spots(next, coords, &cell_count);
        if (found > 0)
            printf("  pass %d: detected %d cell(s) (total so far: %d)\n",
                   passes, found, cell_count);

#if SAVE_STEP_IMAGES
        for (int x = 0; x < BMP_WIDTH; x++)
            for (int y = 0; y < BMP_HEIGTH; y++)
                step_image[x][y][0] = step_image[x][y][1] = step_image[x][y][2] = next[x][y];
        snprintf(step_filename, sizeof(step_filename), "%s/erode_step_%02d.bmp", steps_dir, passes);
        write_bitmap(step_image, step_filename);
#endif

        unsigned char (*tmp)[BMP_HEIGTH] = current;
        current = next;
        next = tmp;
    }
    printf("Eroded to completion after %d pass(es)\n", passes);

    // Step 5
    generate_output_image(color_image, coords, cell_count);

    // Step 6
    write_bitmap(color_image, output_path);
    printf("Detected %d cell(s) total:\n", cell_count);
    for (int i = 0; i < cell_count; i++)
        printf("  cell %d: (x=%d, y=%d)\n", i, coords[i][0], coords[i][1]);
    printf("Wrote output image with %d marked cell(s) to '%s'\n", cell_count, output_path);




    
    return 0;
}