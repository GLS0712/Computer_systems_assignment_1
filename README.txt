This folder contains code and material for the Assignment 1.

If you use the terminal, compile and run 'main.c' as follows. Note: run it in your computer's own terminal (cmd/PowerShell/bash), not the VS Code integrated terminal.

Usage: <program> <input.bmp> [output.bmp]
- <input.bmp> is required: the path to the image you want to run detection on.
- [output.bmp] is optional: the path to write the marked-up result to.

Linux/Mac:
- To compile: gcc cbmp.c main.c -o main.out -std=c99
- To run: ./main.out example.bmp example_out.bmp
- Example with a sample image: ./main.out samples/easy/1EASY.bmp results/1EASY_out.bmp

Windows:
- To compile: gcc cbmp.c main.c -o main.exe -std=c99
- To run: main.exe example.bmp example_out.bmp
- Example with a sample image: main.exe samples/easy/1EASY.bmp results/1EASY_out.bmp

The program prints the run time, and saves the output image (input image with a red cross marker drawn on each detected cell) to the given output path.

The folder 'results' will be filled with the output images produced by running the algorithm.

The folder 'samples' provides you with sample images for the 4 different levels of detection difficulty: easy, medium, hard and impossible.

