#include "./BSP/LEG_ KINEMATICS/LegRestKinematics.h"
#include "arm_math.h"
S_LEGKINEMATICS_PRA legkinematicspra;
/*角度值与推杆长度映射*/
void legKinematics(void)
{
    /*DSP库测试代码*/
    // 	uint8_t i;
    // // 浮点数数组
    // float32_t pDataA[9] = {1.1f, 1.1f, 2.1f, 2.1f, 3.1f, 3.1f, 4.1f, 4.1f, 5.1f};
    // float32_t pDataB[9] = {1001.1f, 1.1f, 2.1f, 2.1f, 3.1f, 3.1f, 4.1f, 4.1f, 5.1f};
    // float32_t pDataDst[9];
    // arm_matrix_instance_f32 pSrcA; //3 行 3 列数据
    // arm_matrix_instance_f32 pSrcB; //3 行 3 列数据
    // arm_matrix_instance_f32 pDst;
    // pSrcA.numCols = 3;
    // pSrcA.numRows = 3;
    // pSrcA.pData = pDataA;
    // pSrcB.numCols = 3;
    // pSrcB.numRows = 3;
    // pSrcB.pData = pDataB;
    // pDst.numCols = 3;
    // pDst.numRows = 3;
    // pDst.pData = pDataDst;
    // arm_mat_mult_f32(&pSrcA, &pSrcB, &pDst);
    // for(i = 0; i < 9; i++)
    // {
    // 	printf("pDataDst[%d] = %f\r\n", i, pDataDst[i]);
    // }

    // 矩阵参数参数映射
   /* theta1_AD = 0.5437*(0.0377272727272727*B1_AD-11.31818181818182) +15.24;
      theta2_AD = 1.119*(0.0377272727272727*B1_AD-11.31818181818182) +23.21;
      theta3_AD = 0.5593*(0.0325379609544469*B2_AD-8.3297) +7.986;
      theta4_AD= 0.6119*(0.0746*A2_AD-147.7) +62.22;
      L4  =0.046886*adcdata.leglength_pos+359.5;
   */
    legkinematicspra.L0 = 0.0;
    legkinematicspra.L1 = 273.0;
    legkinematicspra.L2 = 310.0;
    legkinematicspra.L3 = 88.2;
    legkinematicspra.L4  =0.0461538*adcdata.leglength_pos+353.769246;
    legkinematicspra.theta1 = (180-(0.5437*(0.0377272727272727*adcdata.lift_pos-11.31818181818182) +15.24))*PI/180.0;
    legkinematicspra.theta2 = (180+(1.119*(0.0377272727272727*adcdata.lift_pos-11.31818181818182) +23.21))*PI/180.0;
    legkinematicspra.theta3 = (22.8-(0.5593*(0.0325379609544469*adcdata.pedestal_pos-8.3297) +7.986))*PI/180.0;
    legkinematicspra.theta4 = (180+(0.6119*(0.0746*adcdata.legangle_pos-147.7) +62.22))*PI/180.0;
    legkinematicspra.theta5 = 0;
    /*矩阵初始化及相乘 T1_0 * T2_1*/
    float32_t pDataKinematicT1_0[16] = {arm_cos_f32(legkinematicspra.theta1),-arm_sin_f32(legkinematicspra.theta1),0,0,
                                        arm_sin_f32(legkinematicspra.theta1), arm_cos_f32(legkinematicspra.theta1),0,0,                                                                               
                                        0,0,1,0,0,0,0,1};                                                                                
                                                                                                                     
    float32_t pDataKinematicT2_1[16]  = {arm_cos_f32(legkinematicspra.theta2),-arm_sin_f32(legkinematicspra.theta2),0,legkinematicspra.L1,
                                        arm_sin_f32(legkinematicspra.theta2), arm_cos_f32(legkinematicspra.theta2),0,0,                                                                              
                                        0,0,1,0,0,0,0,1};   
    float32_t pDataDstT2_0[16];
    arm_matrix_instance_f32 pSrcT1_0; // 4 行 4列数据
    arm_matrix_instance_f32 pSrcT2_1; // 4 行 4列数据
    arm_matrix_instance_f32 T2_0pDst;
    pSrcT1_0.numCols = 4;
    pSrcT1_0.numRows = 4;
    pSrcT1_0.pData = pDataKinematicT1_0;
    pSrcT2_1.numCols = 4;
    pSrcT2_1.numRows = 4;
    pSrcT2_1.pData = pDataKinematicT2_1;
    T2_0pDst.numCols = 4;
    T2_0pDst.numRows = 4;
    T2_0pDst.pData = pDataDstT2_0;
    arm_mat_mult_f32(&pSrcT1_0, &pSrcT2_1, &T2_0pDst);
    /*矩阵初始化及相乘 T2_0 * T3_2*/
    float32_t pDataKinematicT3_2[16] = {arm_cos_f32 (legkinematicspra.theta3),-arm_sin_f32(legkinematicspra.theta3),0,legkinematicspra.L2,
                                        arm_sin_f32(legkinematicspra.theta3), arm_cos_f32(legkinematicspra.theta3),0,0,                                                                               
                                        0,0,1,0,0,0,0,1};
    float32_t pDataDstT3_0[16];
    arm_matrix_instance_f32 pSrcT2_0; // 4 行 4列数据
    arm_matrix_instance_f32 pSrcT3_2; // 4 行 4列数据
    arm_matrix_instance_f32 T3_0pDst;
    pSrcT2_0.numCols = 4;
    pSrcT2_0.numRows = 4;
    pSrcT2_0.pData = pDataDstT2_0;
    pSrcT3_2.numCols = 4;
    pSrcT3_2.numRows = 4;
    pSrcT3_2.pData = pDataKinematicT3_2;
    T3_0pDst.numCols = 4;
    T3_0pDst.numRows = 4;
    T3_0pDst.pData = pDataDstT3_0;
    arm_mat_mult_f32(&pSrcT2_0, &pSrcT3_2, &T3_0pDst);
    /*矩阵初始化及相乘 T3_0 * T4_38*/
    float32_t pDataKinematicT4_3[16]={arm_cos_f32(legkinematicspra.theta4),-arm_sin_f32(legkinematicspra.theta4),0,legkinematicspra.L3,
                                        arm_sin_f32(legkinematicspra.theta4),arm_cos_f32(legkinematicspra.theta4),0,0,                                                                               
                                        0,0,1,0,0,0,0,1};
    float32_t pDataDstT4_0[16];
    arm_matrix_instance_f32 pSrcT3_0; // 4 行 4列数据
    arm_matrix_instance_f32 pSrcT4_3; // 4 行 4列数据
    arm_matrix_instance_f32 T4_0pDst;
    pSrcT3_0.numCols = 4;
    pSrcT3_0.numRows = 4;
    pSrcT3_0.pData = pDataDstT3_0;
    pSrcT4_3.numCols = 4;
    pSrcT4_3.numRows = 4;
    pSrcT4_3.pData = pDataKinematicT4_3;
    T4_0pDst.numCols = 4;
    T4_0pDst.numRows = 4;
    T4_0pDst.pData = pDataDstT4_0;
    arm_mat_mult_f32(&pSrcT3_0, &pSrcT4_3, &T4_0pDst);
    /*矩阵初始化及相乘 T4_0 * T5_4*/
    float32_t pDataKinematicT5_4[16] = {1,0,0,legkinematicspra.L4,0,1,0,0,0,0,1,0,0,0,0,1};
    float32_t pDataDstT5_0[16];
    arm_matrix_instance_f32 pSrcT4_0; // 4 行 4列数据
    arm_matrix_instance_f32 pSrcT5_4; // 4 行 4列数据
    arm_matrix_instance_f32 T5_0pDst;
    pSrcT4_0.numCols = 4;
    pSrcT4_0.numRows = 4;
    pSrcT4_0.pData = pDataDstT4_0;
    pSrcT5_4.numCols = 4;
    pSrcT5_4.numRows = 4;
    pSrcT5_4.pData = pDataKinematicT5_4;
    T5_0pDst.numCols = 4;
    T5_0pDst.numRows = 4;
    T5_0pDst.pData = pDataDstT5_0;
    arm_mat_mult_f32(&pSrcT4_0, &pSrcT5_4, &T5_0pDst);

    legkinematicspra.x_o = pDataDstT5_0[3];
    legkinematicspra.y_o = pDataDstT5_0[7];
    legkinematicspra.z_o = pDataDstT5_0[11];
}
