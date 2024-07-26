/**

sdt_dead_reckoning.h v1.0
By Stephan Soller <stephan.soller@helionweb.de>
Implementation of the paper "The dead reckoning signed distance transform" by George J. Grevera
Licensed under the MIT license

QUICK START

	#include ...
	#include ...
	#define SDT_DEAD_RECKONING_IMPLEMENTATION
	#include "sdt_dead_reckoning.h"
	...

	// Load the mask image with stb_image.h
	int width = 0, height = 0;
	uint8_t* mask = stbi_load("mask.png", &width, &height, NULL, 1);

	// Allocate and create the distance field for the mask. Pixels > 16 in the mask are considered inside.
	// Negative distances in the distance field are inside, positive ones outside.
	float* distance_field = malloc(width * height * sizeof(distance_field[0]));
	sdt_dead_reckoning(width, height, 16, mask, distance_field);

	// Create an 8 bit version of the distance field by mapping the distance -128..127 to the brightness 0..255
	uint8_t* distance_field_8bit = malloc(width * height * sizeof(distance_field_8bit[0]));
	for(int n = 0; n < width * height; n++) {
		float mapped_distance = distance_field[n] + 128;
		float clamped_distance = fmaxf(0, fminf(255, mapped_distance))
		distance_field_8bit[n] = clamped_distance;
	}

	// Save the 8 bit distance field into a PNG with stb_image_write.h
	stbi_write_png("out_03.df.png", width, height, 1, distance_field_8bit, 0);

DOCUMENTATION

The library only contains one function:

	void sdt_dead_reckoning(unsigned int width, unsigned int height, unsigned char threshold,  const unsigned char* image, float* distance_field);

- `width` and `height` are the dimensions of the input bitmap and output distance field.
- `threshold` defines which pixels of `image` are interpreted as inside or outside. Pixels greater than `threshold` are
  considered inside, everything else as outside.
- `image` is a pointer to the 8 bit image data, one byte per pixel without padding (width * height bytes in total).
- `distance_field` is a pointer to a float buffer with one float per pixel and no padding (width * hight * sizeof(float)
  bytes in total). It is overwritten with the finished distance field. The distance field is returned as floats so you
  can decide for yourself how to map the field into your target format (e.g. an 8 bit image or half-float).

The function mallocs internal buffers. If that turns out to be a bottleneck feel free to move that out of the function.
The source code is quite short and straight forward (even if the math isn't). A look at the paper might help, too.

The function is an implementation of the paper "The dead reckoning signed distance transform" by George J. Grevera. The
paper contains quite nice pseudo-code of the algorithm. The C++ implementation at http://people.sju.edu/~ggrevera/software/distance.tar.gz
(for comparing different algorithms) was also used to fill in some gaps.

This implementation differs from the paper pseudo-code and C++ implementation in three aspects:

- Negative distances are used to designate the inside. The paper does it the other way around.
- The paper implementation has the symmetry under complement property. Meaning you'll get a distance of 0 outside _and
  inside_ of the outline. This creates to a 2px wide region around the outline where the distance is 0. When you try to
  render that distance field in GLSL the local per-pixel derivative in that region becomes 0 (no change across a pixel).
  And that can screw up anti-aliasing in the shaders. Therefore this implementation disables the symmetry under
  complement property as outlined in the paper and pseudo-code. Thanks to that you can simply add or subtract values
  from the distance to shrink or grow the outline.
- The paper implementation requires that the outermost pixels of the mask are black. It also leaves the outermost values
  of the distance field at infinity. sdt_dead_reckoning() instead does the padding internally and returns a proper
  distance in each pixel of the output distance field (at the cost of padding in the internal buffers).

VERSION HISTORY

v1.0  2018-08-31  Initial release

**/
#ifndef SDT_DEAD_RECKONING_HEADER
#define SDT_DEAD_RECKONING_HEADER

#ifdef __cplusplus
extern "C" {
#endif

void sdt_dead_reckoning(unsigned int width, unsigned int height, unsigned char threshold, const unsigned char* image, float* distance_field);

#ifdef __cplusplus
}
#endif

#endif  // SDT_DEAD_RECKONING_HEADER