# Image Converter

The 128x32 pixel OLED device on the ZedBoard uses a binary file format consisting of 512 bytes. Each pixel is controlled by a single bit, starting at the upper left of the image and the start of the .bin file. TODO: is it big-endian or little-endian?

The perl script ImageGen.pl takes in a [pgm mode 2 image](https://en.wikipedia.org/wiki/Netpbm_format#PGM_example) on `STDIN` and prints to `STDOUT` in the ZedBoard OLED device file format. Note that the color depth must be at least 128, as the script converts all colors with value 128 or greater into 1s, and everything else into 0s.

The script is not robust. The .pgm file it reads must have each pixel on a newline, and a header of size 3. GIMP does nearly this, but includes a comment in the header which must be manually stripped out.

## Example Usage

Create a file Image.pgm using GIMP, or similar. Make sure that it is saved in ASCII mode. Remove the comment in the image header manually, and then run `cat Image.pgm | perl ImageGen.pl > image.bin`.