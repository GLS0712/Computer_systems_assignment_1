//To compile (win): gcc cbmp.c main.c -o main.exe -std=c99
//To run (win): main.exe example.bmp example_inv.bmp

#include <stdlib.h>
#include <stdio.h>
#include "cbmp.h"

void convert_to_grayscale(unsigned char image[BMP_WIDTH][BMP_HEIGTH][BMP_CHANNELS],
                          unsigned char gray[BMP_WIDTH][BMP_HEIGTH]);
void apply_threshold(unsigned char gray[BMP_WIDTH][BMP_HEIGTH],
                     unsigned char binary[BMP_WIDTH][BMP_HEIGTH], int threshold);
void erode_image(unsigned char binary[BMP_WIDTH][BMP_HEIGTH],
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
    static unsigned char output_image[BMP_WIDTH][BMP_HEIGTH][BMP_CHANNELS];

    int threshold = 127;

    read_bitmap(input_path, color_image); // læser billedet
    printf("Loaded '%s' (%d x %d, %d channels)\n",
           input_path, BMP_WIDTH, BMP_HEIGTH, BMP_CHANNELS);

    // flipper farverne til den omvendte altså hvid til sort og sort til hvid
    // for (int x = 0; x < BMP_WIDTH; x++)
    // {
    //     for (int y = 0; y < BMP_HEIGTH; y++)
    //     {
    //         for (int c = 0; c < BMP_CHANNELS; c++)
    //         {
    //             color_image[x][y][c] = 255 - color_image[x][y][c];
    //         }
    //     }
    // }

    convert_to_grayscale(color_image, gray_image);        // updaterer billedet til gray-scale
    apply_threshold(gray_image, binary_image, threshold); // updaterer billedet til sort-hvid

    for (int x = 0; x < BMP_WIDTH; x++)
    {
        for (int y = 0; y < BMP_HEIGTH; y++)
        {
            for (int c = 0; c < BMP_CHANNELS; c++)
            {
                output_image[x][y][c] = binary_image[x][y];
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
            unsigned char value = (gray[x][y] <= threshold) ? 0 : 255;
            for (int c = 0; c < BMP_CHANNELS; c++)
            {
                binary[x][y] = value;
            }
        }
    }
}

void erode_image(unsigned char binary[BMP_WIDTH][BMP_HEIGTH],
                 unsigned char eroded[BMP_WIDTH][BMP_HEIGTH])
{
    for (int x = 0; x < BMP_WIDTH; x++)
    {
        for (int y = 0; y < BMP_HEIGTH; y++)
        {
            if (x == 0 && y == 0)
            {
                if (binary[x + 1][y] == 0 || binary[x][y + 1] == 0)
                {
                    eroded[x][y] = 0;
                }
            }
            else if (x == 255 && y == 255)
            {
                if (binary[x - 1][y] == 0 || binary[x][y - 1] == 0)
                {
                    eroded[x][y] = 0;
                }
            }
            else if (x == 255 && y == 0)
            {
                if (binary[x - 1][y] == 0 || binary[x][y + 1] == 0)
                {
                    eroded[x][y] = 0;
                }
            }
            else if (x == 0 && y == 255)
            {
                if (binary[x + 1][y] == 0 || binary[x][y - 1] == 0)
                {
                    eroded[x][y] = 0;
                }
            }
            else if (x == 0)
            {
                if (binary[x + 1][y] == 0 || binary[x][y + 1] == 0 || binary[x][y - 1] == 0)
                {
                    eroded[x][y] = 0;
                }
            }
            else if (y == 0)
            {
                if (binary[x + 1][y] == 0 || binary[x - 1][y] == 0 || binary[x][y + 1] == 0)
                {
                    eroded[x][y] = 0;
                }
            }
            else if (x == 255)
            {
                if (binary[x - 1][y] == 0 || binary[x][y + 1] == 0 || binary[x][y - 1] == 0)
                {
                    eroded[x][y] = 0;
                }
            }
            else if (y == 255)
            {
                if (binary[x - 1][y] == 0 || binary[x + 1][y] == 0 || binary[x][y + 1] == 0 || binary[x][y - 1] == 0)
                {
                    eroded[x][y] = 0;
                }
            }
            else
            {
                if (binary[x - 1][y] == 0 || binary[x + 1][y] == 0 || binary[x][y + 1] == 0 || binary[x][y - 1] == 0)
                {
                    eroded[x][y] = 0;
                }
            }
        }
    }
}