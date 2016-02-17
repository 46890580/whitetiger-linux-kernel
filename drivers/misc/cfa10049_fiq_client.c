/*
 * Crystalfontz CFA-10049 FIQ test app
 *
 * Copyright (C) 2012 Free Electrons
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "cfa10049_fiq.h"

#define FIQ_PATH	"/dev/cfa10049_fiq"

int main(int argc, char *argv[])
{
	struct fiq_buffer *fiq_buf;
	struct stat st;
	void *fiq_addr, *input_addr;
	int fiq_fd, input_fd;
	int ret = 0, i;
	unsigned long size, start_size;

	input_fd = open(argv[1], O_RDONLY);
	if (!input_fd) {
		ret = errno;
		perror("Couldn't open input file");
		goto out;
	}

	ret = fstat(input_fd, &st);
	if (ret) {
		perror("Couldn't stat the input file");
		goto out_close_input;
	}

	fiq_fd = open(FIQ_PATH, O_RDWR);
	if (!fiq_fd) {
		ret = errno;
		perror("Couldn't open fiq file");
		goto out_close_input;
	}

	input_addr = mmap(NULL, st.st_size + 1, PROT_READ, MAP_SHARED,
			  input_fd, 0);
	if (input_addr == MAP_FAILED) {
		ret = errno;
		perror("Couldn't map the input file");
		goto out_close_fiq;
	}

	fiq_addr = mmap(NULL, FIQ_BUFFER_SIZE, PROT_READ | PROT_WRITE,
			MAP_SHARED, fiq_fd, 0);
	if (fiq_addr == MAP_FAILED) {
		ret = errno;
		perror("Couldn't map the fiq buffer");
		goto out_munmap_input;
	}

	fiq_buf = (struct fiq_buffer*)fiq_addr;

	/*
	 * Compute the size at which we will start the FIQ.
	 * This is basically half of the smallest size between the buffer and
	 * the file size.
	 */
	size = st.st_size / sizeof(struct fiq_cell);
	if (fiq_buf->size < size)
		start_size = fiq_buf->size / 2;
	else
		start_size = size / 2;

	/*
	 * Fill in the buffer with a large file.
	 * We first read the input file, begin to fill the buffer, and
	 * once we are at half the buffer size, start the FIQ.
	 * This will probably need some more calibration to avoid the
	 * over/underruns.
	 */
	for (i = 0; i < size; i++) {
		struct fiq_cell *input_cell, *output_cell;

		/* Check for the underruns */
		if (fiq_buf->status == FIQ_STATUS_ERR_URUN) {
			printf("Underrun, stopping\n");
			ioctl(fiq_fd, FIQ_STOP);
			ret = -ENOMEM;
			goto out_munmap_fiq;
		}

		/* Copy the data to the buffer */
		input_cell = ((struct fiq_cell*)input_addr) + i;
		output_cell = fiq_buf->data + fiq_buf->wr_idx++;

		memcpy(output_cell, input_cell, sizeof(struct fiq_cell));

		if (fiq_buf->wr_idx >= fiq_buf->size) {
			printf("Reached the end of the buffer.... "
			       "Starting back from the beginning of it\n");
			fiq_buf->wr_idx = 0;
		}

		/*
		 * If we reached half the buffer/file size, and if the
		 * FIQ is not already running, start it.
		 */
		if (i >= (start_size / 2) &&
		    fiq_buf->status != FIQ_STATUS_RUNNING) {
			printf("Starting the FIQ...");

			ret = ioctl(fiq_fd, FIQ_START);
			if (ret) {
				printf("Couldn't start the FIQ\n");
				goto out_munmap_fiq;
			}

			printf("\n");
		}
	}

out_munmap_fiq:
	munmap(fiq_addr, FIQ_BUFFER_SIZE);
out_munmap_input:
	munmap(input_addr, st.st_size + 1);
out_close_fiq:
	close(fiq_fd);
out_close_input:
	close(input_fd);
out:
	return ret;
}
