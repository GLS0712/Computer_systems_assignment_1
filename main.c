// kompilering (win): gcc cbmp.c main.c -o main.exe -std=c99
// kørsel (win): main.exe example.bmp [output.bmp]
// skal køres i din computers terminal, ikke vs-code-terminalen
//
// Celle-detektion med distance transform + watershed-agtig top-finding:
//   1. Indlæs billede
//   2. Konvertér til gråtoner
//   3. Binær threshold (Otsus metode)
//   4. Distance transform: hver hvid pixel får sin afstand til den
//      nærmeste sorte pixel -> hver celle bliver en "bakke"
//   5. Find bakketoppe (cellecentre). Bakketoppe uden en dal imellem
//      tilhører den samme celle og bliver slået sammen.
//   6. Tegn en markør på hver celle
//   7. Gem output-billedet og udskriv resultaterne

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

/* ------------------------------------------------------------------ */
/* Parametre                                                           */
/* ------------------------------------------------------------------ */

#define MAX_CELLS 2000
#define MAX_PEAKS 50000      // maks antal bakketop-kandidater før sammenlægning

// Afstande er i chamfer-enheder: 3 = én pixel lige, 4 = én diagonalt.
#define MIN_PEAK_DEPTH 6     // ignorér pletter der er mindre end ~2 px dybe
#define PEAK_RADIUS 5        // en bakketop skal være højest inden for denne radius (px)

#define MIN_CELL_DISTANCE 5  // bakketoppe tættere end dette (px) er altid én celle
#define VALLEY_SEARCH 25     // tjek for dal mellem bakketoppe tættere end dette (px)
#define VALLEY_PERCENT 70    // samme celle hvis linjen mellem to bakketoppe aldrig
                             // falder under denne % af den laveste bakketop

#define DEBUG_VALLEYS 1      // 1 = udskriv hver dal-beslutning (til finjustering)

#define MARKER_ARM_LENGHT 5
#define MARKER_R 255
#define MARKER_G 0
#define MARKER_B 0

/* ------------------------------------------------------------------ */
/* Små hjælpefunktioner                                                */
/* ------------------------------------------------------------------ */

static int min_int(int a, int b) { return a < b ? a : b; }
static int max_int(int a, int b) { return a > b ? a : b; }

// Returnerer afstandsværdien, eller 0 (sort) uden for billedet.
static int dist_at(unsigned char d[BMP_WIDTH][BMP_HEIGTH], int x, int y)
{
    if (x < 0 || x >= BMP_WIDTH || y < 0 || y >= BMP_HEIGTH)
        return 0;
    return d[x][y];
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
/* Trin 2: gråtoner                                                    */
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
/* Trin 3: threshold (Otsus metode)                                    */
/* ------------------------------------------------------------------ */

// Prøver alle thresholds t, deler pixels op i baggrund (gray <= t)
// og forgrund (gray > t), og vælger det t der maksimerer
//   count0 * count1 * (mean0 - mean1)^2
// dvs. den opdeling hvor begge grupper er store OG meget forskellige i lysstyrke.
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

    // Løbende totaler for baggrundssiden, så hvert t er O(1).
    long count0 = 0, sum0 = 0;
    double best_variance = -1.0;
    int best_t = 0;

    for (int t = 0; t < 255; t++)
    {
        count0 += histogram[t];
        sum0 += (long)t * histogram[t];

        long count1 = total_count - count0;
        if (count0 == 0 || count1 == 0)
            continue; // alt ligger på én side, ingen gyldig opdeling

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
/* Trin 4: distance transform                                          */
/* ------------------------------------------------------------------ */

// Erstatter hver hvid pixel med dens afstand til den nærmeste sorte pixel
// (chamfer 3-4: lige skridt = 3, diagonalt skridt = 4 ~ 3*sqrt(2)).
// To gennemløb: forlæns og baglæns. Hvert gennemløb læser kun naboer der
// allerede er blevet konverteret, så det kører in place -> ingen ekstra buffer.
// Pixels uden for billedet tæller som sorte.
void distance_transform(unsigned char img[BMP_WIDTH][BMP_HEIGTH])
{
    // Forlæns gennemløb
    for (int x = 0; x < BMP_WIDTH; x++)
        for (int y = 0; y < BMP_HEIGTH; y++)
        {
            if (img[x][y] == 0)
                continue; // sort forbliver 0
            int d = 255;
            d = min_int(d, dist_at(img, x - 1, y - 1) + 4);
            d = min_int(d, dist_at(img, x - 1, y) + 3);
            d = min_int(d, dist_at(img, x - 1, y + 1) + 4);
            d = min_int(d, dist_at(img, x, y - 1) + 3);
            img[x][y] = (unsigned char)d;
        }

    // Baglæns gennemløb
    for (int x = BMP_WIDTH - 1; x >= 0; x--)
        for (int y = BMP_HEIGTH - 1; y >= 0; y--)
        {
            if (img[x][y] == 0)
                continue;
            int d = img[x][y];
            d = min_int(d, dist_at(img, x + 1, y + 1) + 4);
            d = min_int(d, dist_at(img, x + 1, y) + 3);
            d = min_int(d, dist_at(img, x + 1, y - 1) + 4);
            d = min_int(d, dist_at(img, x, y + 1) + 3);
            img[x][y] = (unsigned char)d;
        }
}

/* ------------------------------------------------------------------ */
/* Trin 5: find cellecentre (bakketoppe)                               */
/* ------------------------------------------------------------------ */

typedef struct
{
    int x, y, depth;
} Peak;

// Sortering: dybeste først. Uafgjort afgøres af position, så resultatet er
// det samme på alle platforme (qsort er ikke stabil).
static int compare_peaks(const void *a, const void *b)
{
    const Peak *pa = (const Peak *)a;
    const Peak *pb = (const Peak *)b;
    if (pa->depth != pb->depth)
        return pb->depth - pa->depth;
    if (pa->x != pb->x)
        return pa->x - pb->x;
    return pa->y - pb->y;
}

// Går den lige linje mellem to bakketoppe og returnerer det laveste
// punkt på den som en procentdel af den laveste bakketop.
//   ~100% -> ingen dal, samme celle
//   lav % -> dyb dal (smal hals), to celler der rører hinanden
static int valley_percent(unsigned char dist[BMP_WIDTH][BMP_HEIGTH],
                          int x0, int y0, int x1, int y1)
{
    int steps = max_int(abs(x1 - x0), abs(y1 - y0));
    int lower_peak = min_int(dist[x0][y0], dist[x1][y1]);
    if (steps == 0 || lower_peak == 0)
        return 100;

    int lowest = 255;
    for (int i = 0; i <= steps; i++)
    {
        int x = x0 + (x1 - x0) * i / steps;
        int y = y0 + (y1 - y0) * i / steps;
        lowest = min_int(lowest, dist[x][y]);
    }
    return lowest * 100 / lower_peak;
}

// Returnerer antallet af fundne celler; deres centre skrives til coords.
int find_cell_centers(unsigned char dist[BMP_WIDTH][BMP_HEIGTH],
                      int coords[MAX_CELLS][2])
{
    static Peak peaks[MAX_PEAKS];
    int peak_count = 0;

    // 1. Saml alle pixels der er højest inden for PEAK_RADIUS.
    for (int x = 0; x < BMP_WIDTH; x++)
        for (int y = 0; y < BMP_HEIGTH; y++)
        {
            int d = dist[x][y];
            if (d < MIN_PEAK_DEPTH)
                continue;

            int is_peak = 1;
            for (int dx = -PEAK_RADIUS; dx <= PEAK_RADIUS && is_peak; dx++)
                for (int dy = -PEAK_RADIUS; dy <= PEAK_RADIUS; dy++)
                    if (dist_at(dist, x + dx, y + dy) > d)
                    {
                        is_peak = 0;
                        break;
                    }
            if (!is_peak)
                continue;

            if (peak_count < MAX_PEAKS)
            {
                peaks[peak_count].x = x;
                peaks[peak_count].y = y;
                peaks[peak_count].depth = d;
                peak_count++;
            }
            else
            {
                fprintf(stderr, "Warning: MAX_PEAKS reached, ignoring peak at (%d,%d)\n", x, y);
            }
        }

    // 2. Dybeste bakketoppe først: det egentlige centrum af hver celle bliver
    //    registreret før en mindre bule på den samme celle.
    qsort(peaks, peak_count, sizeof(Peak), compare_peaks);

    // 3. Acceptér en bakketop som en ny celle, medmindre den tilhører en vi allerede har.
    int cell_count = 0;
    for (int p = 0; p < peak_count; p++)
    {
        int x = peaks[p].x, y = peaks[p].y;
        int same = 0;

        for (int i = 0; i < cell_count && !same; i++)
        {
            int dx = coords[i][0] - x;
            int dy = coords[i][1] - y;
            int d2 = dx * dx + dy * dy;

            if (d2 < MIN_CELL_DISTANCE * MIN_CELL_DISTANCE)
            {
                same = 1; // samme flade bakketop
            }
            else if (d2 < VALLEY_SEARCH * VALLEY_SEARCH)
            {
                int pct = valley_percent(dist, coords[i][0], coords[i][1], x, y);
                if (pct >= VALLEY_PERCENT)
                    same = 1;
#if DEBUG_VALLEYS
                printf("  [valley] peak (%d,%d) vs cell (%d,%d): valley %d%% -> %s\n",
                       x, y, coords[i][0], coords[i][1], pct,
                       pct >= VALLEY_PERCENT ? "SAME cell" : "separate");
#endif
            }
        }

        if (same)
            continue;

        if (cell_count < MAX_CELLS)
        {
            coords[cell_count][0] = x;
            coords[cell_count][1] = y;
            cell_count++;
        }
        else
        {
            fprintf(stderr, "Warning: MAX_CELLS reached, dropping cell at (%d,%d)\n", x, y);
        }
    }
    return cell_count;
}

/* ------------------------------------------------------------------ */
/* Trin 6: tegn markører                                               */
/* ------------------------------------------------------------------ */

void generate_output_image(unsigned char color_image[BMP_WIDTH][BMP_HEIGTH][BMP_CHANNELS],
                           int coords[MAX_CELLS][2], int cell_count)
{
    for (int i = 0; i < cell_count; i++)
    {
        int cx = coords[i][0];
        int cy = coords[i][1];

        // vandret arm
        for (int dx = -MARKER_ARM_LENGHT; dx <= MARKER_ARM_LENGHT; dx++)
        {
            int x = cx + dx;
            if (x < 0 || x >= BMP_WIDTH)
                continue;
            color_image[x][cy][0] = MARKER_R;
            color_image[x][cy][1] = MARKER_G;
            color_image[x][cy][2] = MARKER_B;
        }

        // lodret arm
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
/* main                                                                */
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
    static unsigned char binary_image[BMP_WIDTH][BMP_HEIGTH]; // indeholder senere afstande
    static unsigned char debug_image[BMP_WIDTH][BMP_HEIGTH][BMP_CHANNELS];
    static int coords[MAX_CELLS][2];

    char output_dir[256];
    char steps_dir[300];
    char step_filename[512];
    get_output_folders(output_path, output_dir, sizeof(output_dir),
                       steps_dir, sizeof(steps_dir));
    ensure_directory(output_dir);
    ensure_directory(steps_dir);

    // Trin 1
    read_bitmap(input_path, color_image);
    printf("Loaded '%s' (%d x %d, %d channels)\n",
           input_path, BMP_WIDTH, BMP_HEIGTH, BMP_CHANNELS);

    // Trin 2
    convert_to_grayscale(color_image, gray_image);

    // Trin 3
    int threshold = find_threshold(gray_image);
    printf("Dynamic threshold: %d\n", threshold);
    apply_threshold(gray_image, binary_image, threshold);

    // Trin 4 (in place: binary_image indeholder nu afstande)
    distance_transform(binary_image);

    // Debug-billede af afstandskortet: lysere = dybere inde i en celle.
    for (int x = 0; x < BMP_WIDTH; x++)
        for (int y = 0; y < BMP_HEIGTH; y++)
        {
            int v = binary_image[x][y] * 8;
            unsigned char g = (unsigned char)(v > 255 ? 255 : v);
            debug_image[x][y][0] = debug_image[x][y][1] = debug_image[x][y][2] = g;
        }
    snprintf(step_filename, sizeof(step_filename), "%s/distance_map.bmp", steps_dir);
    write_bitmap(debug_image, step_filename);

    // Trin 5
    int cell_count = find_cell_centers(binary_image, coords);

    // Trin 6
    generate_output_image(color_image, coords, cell_count);

    // Trin 7
    write_bitmap(color_image, output_path);
    printf("Detected %d cell(s) total:\n", cell_count);
    for (int i = 0; i < cell_count; i++)
        printf("  cell %d: (x=%d, y=%d)\n", i, coords[i][0], coords[i][1]);
    printf("Wrote output image with %d marked cell(s) to '%s'\n", cell_count, output_path);
    return 0;
}
