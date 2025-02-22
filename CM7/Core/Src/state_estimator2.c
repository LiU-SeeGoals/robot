#include "arm_math.h"
#include "state_estimator2.h"

#include "log.h"

static LOG_Module internal_log_mod;

const float32_t A_f32[16] =
{
  /* Const,   numTaps,   blockSize,   numTaps*blockSize */
  1.0,     0.0,     0.0,     1.0,
  0.0,     1.0,     0.0,     0.0,
  0.0,     0.0,     1.0,     0.0,
  0.0,     0.0,     0.0,     1.0,
};

const float32_t B_f32[16] =
{
  1.0,     0.0,     0.0,     1.0,
  0.0,     1.0,     0.0,     1.0,
  0.0,     0.0,     1.0,     1.0,
  0.0,     0.0,     0.0,     1.0,
};


/*
 *  Init logging for state
 */

void STATE_Init(){
  LOG_InitModule(&internal_log_mod, "STATE", LOG_LEVEL_TRACE);
}

/*
 *  Print a 4x4 matrix
 */
void STATE_logm44(const float32_t* m44){
  LOG_DEBUG("Matrix 4x4 \r\n%0.2f %0.2f %0.2f %0.2f\r\n%0.2f %0.2f %0.2f %0.2f\r\n%0.2f %0.2f %0.2f %0.2f\r\n%0.2f %0.2f %0.2f %0.2f end\r\n", 
      m44[0*4], m44[0*4 + 1], m44[0*4 + 2], m44[0*4 + 3],
      m44[1*4], m44[1*4 + 1], m44[1*4 + 2], m44[1*4 + 3],
      m44[2*4], m44[2*4 + 1], m44[2*4 + 2], m44[2*4 + 3],
      m44[3*4], m44[3*4 + 1], m44[3*4 + 2], m44[3*4 + 3]
      );
}

void STATE_Test(){


  arm_matrix_instance_f32 A;      /* Matrix A Instance */
  arm_matrix_instance_f32 B;      /* Matrix B Instance */
  arm_matrix_instance_f32 AmB;    /* Matrix A mutliplied with B */

  const uint32_t srcRows = 4;
  const uint32_t srcColumns = 4;
  arm_status status;
  arm_status test_status = ARM_MATH_SUCCESS;

  // Result buffer
  float32_t AmB_f32[16];

  /* Initialise A Matrix Instance with numRows, numCols and data array(A_f32) */
  arm_mat_init_f32(&A, srcRows, srcColumns, (float32_t *)A_f32);
  arm_mat_init_f32(&B, srcRows, srcColumns, (float32_t *)B_f32);
  arm_mat_init_f32(&AmB, srcRows, srcColumns, (float32_t *)AmB_f32);
  status = arm_mat_mult_f32(&A, &B, &AmB);

  // Sanity test check that we understand how the mult works
  if (status != ARM_MATH_SUCCESS || AmB_f32[0*4 + 3] != 2 || AmB_f32[1*4 + 3] != 1 || AmB_f32[2*4 + 3] != 1 || AmB_f32[3*4 + 3] != 1){
    test_status = ARM_MATH_TEST_FAILURE;
  }

  if (test_status == ARM_MATH_SUCCESS){
    LOG_DEBUG("Tjoho all test passed\r\n");
  }
}
