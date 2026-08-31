#include "en_dc.h"
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

/*****************************************************************************
 * Defines
 ****************************************************************************/

#ifndef FALSE
#define FALSE (0)
#endif

#ifndef TRUE
#define TRUE (!FALSE)
#endif

/*****************************************************************************
 * Functions
 ****************************************************************************/

/* Encode
 * This function implements run-length encoding with the following format:
 * - The encoded stream consists of runs: [length][data bytes]
 * - Each run starts with a length byte (1-254)
 * - Length byte indicates how many data bytes follow
 * - If length = 0, it's a special marker (not used in this implementation)
 * - Maximum run length is 254 bytes (0xFE)
 *
 * The encoding algorithm from the header:
 * ENCODE_DST_BUF_LEN_MAX(SRC_LEN) = SRC_LEN + ((SRC_LEN + 253) / 254)
 * This means we add one byte for every 254 data bytes (or fraction thereof)
 */
encode_result frame_encode(void *dst_buf_ptr, size_t dst_buf_len,
                               const void *src_ptr, size_t src_len) {
    encode_result result = {0u, ENCODE_OK};
    const uint8_t *src_read_ptr = (const uint8_t*)src_ptr;
    const uint8_t *src_end_ptr = src_read_ptr + src_len;
    uint8_t *dst_buf_start_ptr = (uint8_t*)dst_buf_ptr;
    uint8_t *dst_buf_end_ptr = dst_buf_start_ptr + dst_buf_len;
    uint8_t *dst_write_ptr = dst_buf_start_ptr;
    size_t bytes_remaining = src_len;
    uint8_t run_length;

    // Validate inputs
    if ((dst_buf_ptr == NULL) || (src_ptr == NULL)) {
        result.status = ENCODE_NULL_POINTER;
        return result;
    }

    // Handle empty input
    if (src_len == 0u) {
        // For empty input, we might need to output a single 0 byte
        // as per the macro ENCODE_DST_BUF_LEN_MAX(0) = 1
        if (dst_buf_len >= 1u) {
            *dst_write_ptr++ = 0u;
            result.out_len = 1u;
        } else {
            result.status = ENCODE_OUT_BUFFER_OVERFLOW;
        }
        return result;
    }

    // Check if output buffer is large enough
    // We need at least: src_len + number_of_runs
    // Each run of up to 254 bytes adds 1 byte for length
    size_t min_required = src_len + ((src_len + 253u) / 254u);
    if (dst_buf_len < min_required) {
        result.status = ENCODE_OUT_BUFFER_OVERFLOW;
        return result;
    }

    // Encode the data in runs
    while (src_read_ptr < src_end_ptr) {
        // Start a new run
        uint8_t *run_start_ptr = dst_write_ptr + 1; // Leave space for length byte
        run_length = 0u;
        
        // Copy up to 254 bytes for this run
        while (src_read_ptr < src_end_ptr && run_length < 254u) {
            *dst_write_ptr++ = *src_read_ptr++;
            run_length++;
        }
        
        // Write the length byte at the start of this run
        uint8_t *len_ptr = run_start_ptr - 1;
        *len_ptr = run_length;
    }

    result.out_len = (size_t)(dst_write_ptr - dst_buf_start_ptr);

    // Double-check we didn't overflow
    if (dst_write_ptr > dst_buf_end_ptr) {
        result.status = ENCODE_OUT_BUFFER_OVERFLOW;
        return result;
    }

    return result;
}

/* Decode
 * This function decodes the run-length encoded data
 * The format is: [length][data bytes] for each run
 * - Length byte indicates how many data bytes follow
 * - Maximum run length is 254 bytes
 *
 * The decoding algorithm from the header:
 * DECODE_DST_BUF_LEN_MAX(SRC_LEN) = SRC_LEN - 1 (if SRC_LEN > 0)
 * This means we lose one byte for every run header
 */
decode_result frame_decode(void *dst_buf_ptr, size_t dst_buf_len,
                               const void *src_ptr, size_t src_len) {
    decode_result result = {0u, DECODE_OK};
    const uint8_t *src_read_ptr = (const uint8_t*)src_ptr;
    const uint8_t *src_end_ptr = src_read_ptr + src_len;
    uint8_t *dst_buf_start_ptr = (uint8_t*)dst_buf_ptr;
    uint8_t *dst_buf_end_ptr = dst_buf_start_ptr + dst_buf_len;
    uint8_t *dst_write_ptr = dst_buf_start_ptr;
    size_t remaining_bytes;
    uint8_t run_length;
    size_t i;

    // Validate inputs
    if ((dst_buf_ptr == NULL) || (src_ptr == NULL)) {
        result.status = DECODE_NULL_POINTER;
        return result;
    }

    // Handle empty input
    if (src_len == 0u) {
        result.out_len = 0u;
        return result;
    }

    // Check if we have at least a length byte
    if (src_len < 1u) {
        result.status = DECODE_INPUT_TOO_SHORT;
        return result;
    }

    // Process each run
    while (src_read_ptr < src_end_ptr) {
        // Read the run length
        run_length = *src_read_ptr++;
        
        // Check for zero length (special case - just skip)
        if (run_length == 0u) {
            result.status |= DECODE_ZERO_BYTE_IN_INPUT;
            continue; // Skip this zero-length run
        }

        // Check if we have enough input bytes
        remaining_bytes = (size_t)(src_end_ptr - src_read_ptr);
        if (run_length > remaining_bytes) {
            result.status |= DECODE_INPUT_TOO_SHORT;
            // Decode what we can
            run_length = (uint8_t)remaining_bytes;
        }

        // Check if we have enough output buffer space
        remaining_bytes = (size_t)(dst_buf_end_ptr - dst_write_ptr);
        if (run_length > remaining_bytes) {
            result.status |= DECODE_OUT_BUFFER_OVERFLOW;
            // Only decode what fits
            run_length = (uint8_t)remaining_bytes;
        }

        // Copy the data bytes to output
        for (i = 0; i < run_length; i++) {
            uint8_t src_byte = *src_read_ptr++;
            *dst_write_ptr++ = src_byte;
            
            // Check for zero byte in input (allowed in data)
            if (src_byte == 0u) {
                // Zero bytes are valid data, just note it
                result.status |= DECODE_ZERO_BYTE_IN_INPUT;
            }
        }

        // Check if we've reached the end of input
        if (src_read_ptr >= src_end_ptr) {
            break;
        }
    }

    result.out_len = (size_t)(dst_write_ptr - dst_buf_start_ptr);

    // Check if we decoded anything
    if (result.out_len == 0u && src_len > 0u) {
        // We read input but produced no output - this is suspicious
        if (src_len > 0u) {
            // There was at least some input, so we should have output
            result.status |= DECODE_INPUT_TOO_SHORT;
        }
    }

    return result;
}
