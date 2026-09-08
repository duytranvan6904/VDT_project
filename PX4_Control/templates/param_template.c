/*
 * Template: khai bao tham so tuy chinh moi trong PX4.
 * Dat file nay (hoac them vao file *_params.c da co san trong
 * src/modules/mc_pos_control/) roi build lai — PX4 se tu sinh
 * entry trong QGroundControl > Parameters, khong can code them gi khac.
 */

/**
 * He so bo sung cho vong PID van toc ngang (custom).
 *
 * @group Multicopter Position Control
 * @min 0.0
 * @max 2.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(MPC_CUSTOM_GAIN, 1.0f);

/**
 * Bat/tat dieu chinh tuy chinh cho vong PID van toc (0 = tat, dung PID goc).
 *
 * @group Multicopter Position Control
 * @boolean
 */
PARAM_DEFINE_INT32(MPC_CUSTOM_EN, 0);