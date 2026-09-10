//To compile (win): gcc cbmp.c main.c -o main.exe -std=c99
//To run (win): main.exe example.bmp example_inv.bmp
//has to be in your computer terminal not the vs-code one


#include <stdlib.h>
#include <stdio.h>
#include "cbmp.h"

void convert_to_grayscale(unsigned char image[BMP_WIDTH][BMP_HEIGTH][BMP_CHANNELS],
                          unsigned char gray[BMP_WIDTH][BMP_HEIGTH]);
void apply_threshold(unsigned char gray[BMP_WIDTH][BMP_HEIGTH],
                     unsigned char binary[BMP_WIDTH][BMP_HEIGTH], int threshold);
int erode_image(unsigned char binary[BMP_WIDTH][BMP_HEIGTH],
                 unsigned char eroded[BMP_WIDTH][BMP_HEIGTH]);

int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        fprintf(stderr, "Usage: %s <input.bmp> <output.bmp>\n", argv[0]);
        return 1;
    }
    char *input_path = argv[1];
    char *output_path = argv[2];

    static unsigned char color_image[BMP_WIDTH][BMP_HEIGTH][BMP_CHANNELS];

    static unsigned char gray_image[BMP_WIDTH][BMP_HEIGTH];
    static unsigned char binary_image[BMP_WIDTH][BMP_HEIGTH];
    static unsigned char eroded_image[BMP_WIDTH][BMP_HEIGTH];

    static unsigned char output_image[BMP_WIDTH][BMP_HEIGTH][BMP_CHANNELS];

    int threshold = 127;

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
    char step_filename[256];
    while (changed) {
        // kør en erosion: "current" er input, "next" er output
        // changed bliver 0 når et helt pass ikke fjerner flere pixels (billedet er stabilt)
        changed = erode_image(current, next);
        passes++;

        for (int x = 0; x < BMP_WIDTH; x++) {
            for (int y = 0; y < BMP_HEIGTH; y++) {
                for (int c = 0; c < BMP_CHANNELS; c++) {
                    output_image[x][y][c] = next[x][y];
                }
            }
        }

        // gemmer et billede af hvert enkelt skridt, så man kan se erosionen udvikle sig
        snprintf(step_filename, sizeof(step_filename), "results_example/erode_step_%02d.bmp", passes);
        write_bitmap(output_image, step_filename);

        // byt om på current og next, så næste iteration eroderer videre på det nye billede
        unsigned char (*tmp)[BMP_HEIGTH] = current;
        current = next;
        next = tmp;
    }
    printf("Eroded to completion after %d pass(es)\n", passes);


    for (int x = 0; x < BMP_WIDTH; x++)
    {
        for (int y = 0; y < BMP_HEIGTH; y++)
        {
            for (int c = 0; c < BMP_CHANNELS; c++)
            {
                output_image[x][y][c] = current[x][y];
            }
        }
    }

    write_bitmap(output_image, output_path);
    printf("Wrote thresholded image to '%s'\n", output_path);
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