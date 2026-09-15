//to compile (win): gcc cbmp.c main.c -o main.exe -std=c99
//to run (win): main.exe example.bmp
//skal køres i din computers terminal, ikke vs-code-terminalen


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "cbmp.h"

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

void convert_to_grayscale(unsigned char image[BMP_WIDTH][BMP_HEIGTH][BMP_CHANNELS],
                          unsigned char gray[BMP_WIDTH][BMP_HEIGTH]);
void apply_threshold(unsigned char gray[BMP_WIDTH][BMP_HEIGTH],
                     unsigned char binary[BMP_WIDTH][BMP_HEIGTH], int threshold);
int erode_image(unsigned char binary[BMP_WIDTH][BMP_HEIGTH],
                 unsigned char eroded[BMP_WIDTH][BMP_HEIGTH]);


#define CAPTURE_SIZE 12
#define EXCLUSION_FRAME 1
#define MAX_CELLS 2000
int detect_spots(unsigned char binary[BMP_WIDTH][BMP_HEIGTH],
                  int coords[MAX_CELLS][2], int *cell_count);


#define MARKER_ARM_LENGHT 5
#define MARKER_R 255
#define MARKER_G 0
#define MARKER_B 0
void generate_output_image(unsigned char color_image[BMP_WIDTH][BMP_HEIGTH][BMP_CHANNELS],
                            int coords[MAX_CELLS][2], int cell_count);


// Creates a directory if it doesn't already exist. Safe to call on a path
// that already exists (that's not treated as an error), and safe to call
// with an empty string (meaning "current directory", so nothing to do).
void ensure_directory(const char *path) {
    if (path[0] == '\0') return;
    if (MKDIR(path) != 0 && errno != EEXIST) {
        fprintf(stderr, "Warning: could not create directory '%s'\n", path);
    }
}

// Builds a default output path from the input path alone, e.g.
// "samples/easy/1EASY.bmp" -> "results_example/1EASY_out.bmp". Used when the
// caller doesn't specify an output path explicitly; output always lands in
// results_example regardless of where the input file lives.
void get_default_output_path(const char *input_path, char *output_path, size_t output_path_size) {
    const char *last_slash = strrchr(input_path, '/');
    const char *last_bslash = strrchr(input_path, '\\');
    const char *sep = last_slash;
    if (last_bslash && (!sep || last_bslash > sep)) sep = last_bslash;

    const char *base = sep ? sep + 1 : input_path;

    char name[256];
    strncpy(name, base, sizeof(name) - 1);
    name[sizeof(name) - 1] = '\0';
    char *dot = strrchr(name, '.');
    if (dot) *dot = '\0';

    snprintf(output_path, output_path_size, "results_example/%s_out.bmp", name);
}

// Given an output path like "results_example/1EASY_out.bmp" (or
// "results_example\\1EASY_out.bmp" on Windows), works out:
//   - output_dir: the folder the final output file itself needs, e.g.
//     "results_example" (empty string if output_path has no folder part).
//   - steps_dir: a dedicated "<name>_steps" subfolder for THIS image's
//     per-erosion-pass debug files, e.g. "results_example/1EASY_out_steps".
// A separate steps_dir per image means running this on a whole folder of
// samples never mixes one image's step files up with another's.
void get_output_folders(const char *output_path,
                         char *output_dir, size_t output_dir_size,
                         char *steps_dir, size_t steps_dir_size) {
    const char *last_slash = strrchr(output_path, '/');
    const char *last_bslash = strrchr(output_path, '\\');
    const char *sep = last_slash;
    if (last_bslash && (!sep || last_bslash > sep)) sep = last_bslash;

    char base[256];
    if (sep) {
        size_t dir_len = (size_t)(sep - output_path);
        if (dir_len >= output_dir_size) dir_len = output_dir_size - 1;
        memcpy(output_dir, output_path, dir_len);
        output_dir[dir_len] = '\0';
        strncpy(base, sep + 1, sizeof(base) - 1);
        base[sizeof(base) - 1] = '\0';
    } else {
        output_dir[0] = '\0';
        strncpy(base, output_path, sizeof(base) - 1);
        base[sizeof(base) - 1] = '\0';
    }

    // Fjern filendelsen, f.eks. "1EASY_out.bmp" -> "1EASY_out".
    char *dot = strrchr(base, '.');
    if (dot) *dot = '\0';

    if (output_dir[0] != '\0') {
        snprintf(steps_dir, steps_dir_size, "%s/%s_steps", output_dir, base);
    } else {
        snprintf(steps_dir, steps_dir_size, "%s_steps", base);
    }
}

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
    if (argc == 3) {
        output_path = argv[2];
    } else {
        get_default_output_path(input_path, default_output_path, sizeof(default_output_path));
        output_path = default_output_path;
    }

    static unsigned char color_image[BMP_WIDTH][BMP_HEIGTH][BMP_CHANNELS];

    static unsigned char gray_image[BMP_WIDTH][BMP_HEIGTH];
    static unsigned char binary_image[BMP_WIDTH][BMP_HEIGTH];
    static unsigned char eroded_image[BMP_WIDTH][BMP_HEIGTH];

    static unsigned char output_image[BMP_WIDTH][BMP_HEIGTH][BMP_CHANNELS];

    static int coords[MAX_CELLS][2];
    int cell_count = 0;
    int threshold = 90;

    // Work out (and create) the folders this run needs, up front: output_dir
    // for the final output file, and a dedicated steps_dir for this image's
    // per-erosion-pass debug files. Doing this before any processing means a
    // batch run over many samples fails fast if a path is unwritable.
    char output_dir[256];
    char steps_dir[300];
    get_output_folders(output_path, output_dir, sizeof(output_dir),
                        steps_dir, sizeof(steps_dir));
    ensure_directory(output_dir);
    ensure_directory(steps_dir);

    read_bitmap(input_path, color_image); // læser billedet
    printf("Loaded '%s' (%d x %d, %d channels)\n",
           input_path, BMP_WIDTH, BMP_HEIGTH, BMP_CHANNELS);


    convert_to_grayscale(color_image, gray_image);        // updaterer billedet til gray-scale
    apply_threshold(gray_image, binary_image, threshold); // updaterer billedet til sort-hvid


    // "current" og "next" er pointere til de to buffere (binary_image og eroded_image),
    // så vi kan skifte mellem dem uden at kopiere hele billedet hver gang (ping-pong buffering)
    unsigned char (*current)[BMP_HEIGTH] = binary_image;
    unsigned char (*next)[BMP_HEIGTH] = eroded_image;

    int changed = 1;
    int passes = 0;
    char step_filename[512]; // større end steps_dir, så snprintf aldrig kan afkorte
    while (changed) {
        // kør en erosion: "current" er input, "next" er output
        // changed bliver 0 når et helt pass ikke fjerner flere pixels (billedet er helt sort)
        changed = erode_image(current, next);
        passes++;


        int found = detect_spots(next, coords, &cell_count);
        if (found > 0) {
            printf("  pass %d: detected %d cell(s) (total so far: %d)\n",
                   passes, found, cell_count);
        }


        //ændre billedet til at have det korrekte antal dimentioner
        for (int x = 0; x < BMP_WIDTH; x++) {
            for (int y = 0; y < BMP_HEIGTH; y++) {
                for (int c = 0; c < BMP_CHANNELS; c++) {
                    output_image[x][y][c] = next[x][y];
                }
            }
        }

        // gemmer et billede af hvert enkelt skridt, så man kan se erosionen udvikle sig
        // -- nu inde i steps_dir, som blev oprettet automatisk foroven
        snprintf(step_filename, sizeof(step_filename), "%s/erode_step_%02d.bmp", steps_dir, passes);
        write_bitmap(output_image, step_filename);

        // byt om på current og next, så næste iteration eroderer videre på det nye billede
        unsigned char (*tmp)[BMP_HEIGTH] = current;
        current = next;
        next = tmp;
    }
    printf("Eroded to completion after %d pass(es)\n", passes);
    printf("Detected %d cell(s) total:\n", cell_count);
    for (int i = 0; i < cell_count; i++) {
        printf("  cell %d: (x=%d, y=%d)\n", i, coords[i][0], coords[i][1]);
    }


    generate_output_image(color_image, coords, cell_count);

    write_bitmap(color_image, output_path);
    printf("Wrote output image with %d marked cell(s) to '%s'\n", cell_count, output_path);
    return 0;
}

void convert_to_grayscale(unsigned char image[BMP_WIDTH][BMP_HEIGTH][BMP_CHANNELS],
                          unsigned char gray[BMP_WIDTH][BMP_HEIGTH])
{
    for (int x = 0; x < BMP_WIDTH; x++)
    {
        for (int y = 0; y < BMP_HEIGTH; y++)
        {
            // gennemsnit af rød, grøn og blå -> ét gråtone-tal pr. pixel
            int sum = image[x][y][0] + image[x][y][1] + image[x][y][2];
            gray[x][y] = (unsigned char)(sum / 3);
        }
    }
}

void apply_threshold(unsigned char gray[BMP_WIDTH][BMP_HEIGTH],
                     unsigned char binary[BMP_WIDTH][BMP_HEIGTH], int threshold)
{
    for (int x = 0; x < BMP_WIDTH; x++)
    {
        for (int y = 0; y < BMP_HEIGTH; y++)
        {
            // under/lig threshold bliver sort (0), resten bliver hvid (255)
            unsigned char value = (gray[x][y] <= threshold) ? 0 : 255;
            for (int c = 0; c < BMP_CHANNELS; c++)
            {
                binary[x][y] = value;
            }
        }
    }
}

int erode_image(unsigned char binary[BMP_WIDTH][BMP_HEIGTH],
                 unsigned char eroded[BMP_WIDTH][BMP_HEIGTH])
{
    int changed = 0; // sættes til 1 hvis mindst én pixel bliver eroderet væk i dette pass

    for (int x = 0; x < BMP_WIDTH; x++) {
        for (int y = 0; y < BMP_HEIGTH; y++) {

            // en pixel der allerede er sort forbliver sort
            if (binary[x][y] == 0) {
                eroded[x][y] = 0;
                continue;
            }

            // en hvid pixel overlever kun hvis den IKKE ligger på billedets kant,
            // og alle 4 naboer (op/ned/venstre/højre) også er hvide
            int survives =
                (x > 0) && (x < BMP_WIDTH - 1) && //tjekker at vi er inden for billedet på x-aksen
                (y > 0) && (y < BMP_HEIGTH - 1) && //tjekker at vi er inden for billedet på y-aksen
                binary[x - 1][y] == 255 &&
                binary[x + 1][y] == 255 &&
                binary[x][y - 1] == 255 &&
                binary[x][y + 1] == 255;

            if (survives) {
                eroded[x][y] = 255;
            } else {
                // pixel lå på kanten af en hvid region og bliver "spist" væk
                eroded[x][y] = 0;
                changed = 1;
            }
        }
    }
    return changed;
}

int detect_spots(unsigned char binary[BMP_WIDTH][BMP_HEIGTH],
                  int coords[MAX_CELLS][2], int *cell_count)
{
    int detections_found = 0;
    int half_before_center = CAPTURE_SIZE / 2;
    int half_after_center = CAPTURE_SIZE / 2;

    for (int pixel_x = 0; pixel_x < BMP_WIDTH; pixel_x++) {
        for (int pixel_y = 0; pixel_y < BMP_HEIGTH; pixel_y++) {

            int capture_left = pixel_x - half_before_center, capture_right = pixel_x + half_after_center;
            int capture_top = pixel_y - half_before_center,  capture_bottom = pixel_y + half_after_center;
            int window_left = capture_left - EXCLUSION_FRAME, window_right = capture_right + EXCLUSION_FRAME;
            int window_top = capture_top - EXCLUSION_FRAME,   window_bottom = capture_bottom + EXCLUSION_FRAME;

            // Spring kandidater over, hvis deres vindue ville falde uden for billedet.
            if (window_left < 0 || window_right >= BMP_WIDTH || window_top < 0 || window_bottom >= BMP_HEIGTH) {
                continue;
            }

            // Betingelse 1: mindst én hvid pixel inde i capture-området.
            int found_white_pixel = 0;
            for (int capture_x = capture_left; capture_x <= capture_right && !found_white_pixel; capture_x++) {
                for (int capture_y = capture_top; capture_y <= capture_bottom; capture_y++) {
                    if (binary[capture_x][capture_y] == 255) {
                        found_white_pixel = 1;
                        break;
                    }
                }
            }
            if (!found_white_pixel) continue;

            // Betingelse 2: hver pixel i den omkringliggende exclusion-ring
            // (vinduet minus capture-området) er sort.
            int exclusion_ring_is_black = 1;
            for (int window_x = window_left; window_x <= window_right && exclusion_ring_is_black; window_x++) {
                for (int window_y = window_top; window_y <= window_bottom; window_y++) {
                    int inside_capture_area = (window_x >= capture_left && window_x <= capture_right &&
                                                window_y >= capture_top && window_y <= capture_bottom);
                    if (!inside_capture_area && binary[window_x][window_y] == 255) {
                        exclusion_ring_is_black = 0;
                        break;
                    }
                }
            }
            if (!exclusion_ring_is_black) continue;

            // Detektion! Registrer den, og gør derefter capture-området sort,
            // så den samme celle ikke kan udløse en detektion igen.
            if (*cell_count < MAX_CELLS) {
                coords[*cell_count][0] = pixel_x;
                coords[*cell_count][1] = pixel_y;
                (*cell_count)++;
                detections_found++;
            } else {
                fprintf(stderr, "Warning: MAX_CELLS reached, dropping detection at (%d,%d)\n", pixel_x, pixel_y);
            }

            for (int capture_x = capture_left; capture_x <= capture_right; capture_x++) {
                for (int capture_y = capture_top; capture_y <= capture_bottom; capture_y++) {
                    binary[capture_x][capture_y] = 0;
                }
            }
        }
    }

    return detections_found;
}



void generate_output_image(unsigned char color_image[BMP_WIDTH][BMP_HEIGTH][BMP_CHANNELS],
                            int coords[MAX_CELLS][2], int cell_count)
{
    for (int i = 0; i < cell_count; i++) {
        int cx = coords[i][0];
        int cy = coords[i][1];

        // horizontal arm (varies x, fixed y = cy)
        for (int dx = -MARKER_ARM_LENGHT; dx <= MARKER_ARM_LENGHT; dx++) {
            int x = cx + dx;
            if (x < 0 || x >= BMP_WIDTH) continue;
            color_image[x][cy][0] = MARKER_R;
            color_image[x][cy][1] = MARKER_G;
            color_image[x][cy][2] = MARKER_B;
        }

        // vertical arm (varies y, fixed x = cx)
        for (int dy = -MARKER_ARM_LENGHT; dy <= MARKER_ARM_LENGHT; dy++) {
            int y = cy + dy;
            if (y < 0 || y >= BMP_HEIGTH) continue;
            color_image[cx][y][0] = MARKER_R;
            color_image[cx][y][1] = MARKER_G;
            color_image[cx][y][2] = MARKER_B;
        }
    }
}