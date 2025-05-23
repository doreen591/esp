// Copyright (c) 2011-2024 Columbia University, System Level Design Group
// SPDX-License-Identifier: Apache-2.0

#ifndef __CONV2D_DIRECTIVES_HPP__
#define __CONV2D_DIRECTIVES_HPP__

#if (DMA_WIDTH == 32)
    #define DMA_BEAT_PER_WORD      1
    #define DMA_WORD_PER_BEAT      1
    #define DMA_BEAT_PER_WORD_LOG2 0
    #define DMA_WORD_PER_BEAT_LOG2 0
    #define PLM_IN_NAME            "conv2d_plm_block_in_dma32"
    #define PLM_WEIGHTS_NAME       "conv2d_plm_block_weights_dma32"
    #define PLM_OUT_NAME           "conv2d_plm_block_out_dma64"
#elif (DMA_WIDTH == 64)
    #define DMA_BEAT_PER_WORD      1
    #define DMA_WORD_PER_BEAT      2
    #define DMA_BEAT_PER_WORD_LOG2 0
    #define DMA_WORD_PER_BEAT_LOG2 1
    #define PLM_IN_NAME            "conv2d_plm_block_in_dma64"
    #define PLM_WEIGHTS_NAME       "conv2d_plm_block_weights_dma64"
    #define PLM_OUT_NAME           "conv2d_plm_block_out_dma64"
#endif

#if defined(STRATUS_HLS)

    #define HLS_MAP_plm(_mem, _plm_block_name) HLS_MAP_TO_MEMORY(_mem, _plm_block_name)

    #define HLS_PROTO(_s) HLS_DEFINE_PROTOCOL(_s)

    #define HLS_FLAT(_a) HLS_FLATTEN_ARRAY(_a);

    #define HLS_BREAK_DEP(_a) HLS_BREAK_ARRAY_DEPENDENCY(_a)

    #define HLS_UNROLL_SIMPLE HLS_UNROLL_LOOP(ON)

    #if defined(HLS_DIRECTIVES_BASIC)

    #else

        #error Unsupported or undefined HLS configuration

    #endif /* HLS_DIRECTIVES_* */

#else /* !STRATUS_HLS */

    #define HLS_MAP_plm(_mem, _plm_block_name)
    #define HLS_PROTO(_s)
    #define HLS_FLAT(_a)
    #define HLS_BREAK_DEP(_a)
    #define HLS_UNROLL_SIMPLE

#endif /* STRATUS_HLS */

// floating/fixed point conversions
#define INT2FP(x) int2fp<FPDATA, WORD_SIZE>(x)
#define FP2INT(x) fp2int<FPDATA, WORD_SIZE>(x)

#endif /* __CONV2D_DIRECTIVES_HPP_ */
