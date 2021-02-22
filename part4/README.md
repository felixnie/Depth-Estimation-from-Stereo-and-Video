- img_original_disparity:

The initialized images. Only half of the images (0 - 70) are kept considering the file size.

- img_sum_up_process:

Show how adding up pc term benifits the process. Only several frames are kept considering the file size.

- init/initialized_img_mat:

Initialized images and data (in .mat).

- init/optimized_mat_2nd_iteration:

The data (in .mat) after 2 iterations.

- init:

Where the new bundle optimization results will be places. Previous results may be covered.

So if you need to run on the initialized data, place the files in init/initialized_img_mat in this folder.

- optimized:

The result images after 2 iters. The corresponding .mat data is in init/optimized_mat_2nd_iteration.
