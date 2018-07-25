#include "tst_quaterniontest.h"

#include <quatutils.h>

TEST_F(QuaternionTest, IsConstructible)
{
    Quaternion q_default;

    Vector3D vector_part;
    vector_part << -1.0, 0.0, 1.0;
    Quaternion q_from_vector_part{ vector_part };

    vector_form vector_f;
    vector_f << 0.0, vector_part;
    Quaternion q_from_vector_form{ vector_f };
}

TEST_F(QuaternionTest, IsScalarPartCorrect)
{
    ASSERT_DOUBLE_EQ(10.0, q.scalar_part());
}

TEST_F(QuaternionTest, IsVectorPartCorrect)
{
    auto vector_part = q.vector_part();
    EXPECT_DOUBLE_EQ(-5.0, vector_part[0]);
    EXPECT_DOUBLE_EQ(15.0, vector_part[1]);
    EXPECT_DOUBLE_EQ(2.0, vector_part[2]);
}

TEST_F(QuaternionTest, IsVectorFormCorrect)
{
    auto vf = static_cast<vector_form>(q);
    EXPECT_DOUBLE_EQ(10.0, vf[0]);
    EXPECT_DOUBLE_EQ(-5.0, vf[1]);
    EXPECT_DOUBLE_EQ(15.0, vf[2]);
    EXPECT_DOUBLE_EQ( 2.0, vf[3]);
}

TEST_F(QuaternionTest, IsNormCorrect)
{
    ASSERT_NEAR(18.8149, q.norm(), 1e-4);
}

TEST_F(QuaternionTest, IsNormalizationCorrect)
{
    ASSERT_DOUBLE_EQ(1.0, q.normalize().norm());
}

TEST_F(QuaternionTest, IsRPYCorrect)
{
    auto rpy = q.normalize().rpy();
    EXPECT_NEAR( 1.9988, rpy[0], 1e-4);
    EXPECT_NEAR(-0.1132, rpy[1], 1e-4);
    EXPECT_NEAR(-0.5707, rpy[2], 1e-4);
}

TEST_F(QuaternionTest, IsDCMCorrect)
{
    auto DCM = q.normalize().dcm();
    EXPECT_NEAR(-0.2938, DCM(0, 0), 1e-4);
    EXPECT_NEAR(-0.5367, DCM(0, 1), 1e-4);
    EXPECT_NEAR( 0.7910, DCM(0, 2), 1e-4);
    EXPECT_NEAR(-0.3107, DCM(1, 0), 1e-4);
    EXPECT_NEAR( 0.8362, DCM(1, 1), 1e-4);
    EXPECT_NEAR( 0.4520, DCM(1, 2), 1e-4);
    EXPECT_NEAR(-0.9040, DCM(2, 0), 1e-4);
    EXPECT_NEAR(-0.1130, DCM(2, 1), 1e-4);
    EXPECT_NEAR(-0.4124, DCM(2, 2), 1e-4);
}

TEST_F(QuaternionTest, IsDCMTrCorrect)
{
    auto DCM = q.normalize().dcm();
    auto DCM_TR = q.dcm_tr();

    EXPECT_NEAR(-0.2938, DCM_TR(0, 0), 1e-4);
    EXPECT_NEAR( 0.8362, DCM_TR(1, 1), 1e-4);
    EXPECT_NEAR(-0.4124, DCM_TR(2, 2), 1e-4);

    EXPECT_DOUBLE_EQ(DCM(0, 1), DCM_TR(1, 0));
    EXPECT_DOUBLE_EQ(DCM(0, 2), DCM_TR(2, 0));
    EXPECT_DOUBLE_EQ(DCM(1, 2), DCM_TR(2, 1));
}

TEST_F(QuaternionTest, IsDeltaMtxCorrect)
{
    auto K = q.delta_mtx(0.5);
    EXPECT_NEAR(-2.5, K(0, 0), 1e-2);
    EXPECT_NEAR( 7.5, K(0, 1), 1e-2);
    EXPECT_NEAR( 1.0, K(0, 2), 1e-2);
    EXPECT_NEAR(-5.0, K(1, 0), 1e-2);
    EXPECT_NEAR( 1.0, K(1, 1), 1e-2);
    EXPECT_NEAR(-7.5, K(1, 2), 1e-2);
    EXPECT_NEAR(-1.0, K(2, 0), 1e-2);
    EXPECT_NEAR(-5.0, K(2, 1), 1e-2);
    EXPECT_NEAR(-2.5, K(2, 2), 1e-2);
    EXPECT_NEAR( 7.5, K(3, 0), 1e-2);
    EXPECT_NEAR( 2.5, K(3, 1), 1e-2);
    EXPECT_NEAR(-5.0, K(3, 2), 1e-2);
}

TEST_F(QuaternionTest, IsConjugateCorrect)
{
    auto q_conj = q.conjugate();
    auto q_vector_part = q.vector_part();
    auto conj_vector_part = q_conj.vector_part();

    EXPECT_DOUBLE_EQ(q.scalar_part(), q_conj.scalar_part());
    EXPECT_DOUBLE_EQ(q_vector_part[0], -conj_vector_part[0]);
    EXPECT_DOUBLE_EQ(q_vector_part[1], -conj_vector_part[1]);
    EXPECT_DOUBLE_EQ(q_vector_part[2], -conj_vector_part[2]);
}

TEST_F(QuaternionTest, IsIsRealCorrect)
{
    EXPECT_FALSE(q.is_real());
    EXPECT_TRUE(Quaternion().is_real());
}

TEST_F(QuaternionTest, IsIsPureImageCorrect)
{
    EXPECT_FALSE(q.is_pure_imag());

    vector_form vf;
    vf << 0, Vector3D::Zero();

    EXPECT_FALSE(Quaternion(vf).is_pure_imag());

    vf[1] = 1.0;

    EXPECT_TRUE(Quaternion(vf).is_pure_imag());
}

TEST_F(QuaternionTest, Check_ddcm_dqs_tr)
{
    auto M = q.ddcm_dqs_tr();
    EXPECT_DOUBLE_EQ( 20, M(0, 0));
    EXPECT_DOUBLE_EQ( 4,  M(0, 1));
    EXPECT_DOUBLE_EQ(-30, M(0, 2));
    EXPECT_DOUBLE_EQ(-4,  M(1, 0));
    EXPECT_DOUBLE_EQ( 20, M(1, 1));
    EXPECT_DOUBLE_EQ(-10, M(1, 2));
    EXPECT_DOUBLE_EQ( 30, M(2, 0));
    EXPECT_DOUBLE_EQ( 10, M(2, 1));
    EXPECT_DOUBLE_EQ( 20, M(2, 2));
}

TEST_F(QuaternionTest, Check_ddcm_dqx_tr)
{
    auto M = q.ddcm_dqx_tr();
    EXPECT_DOUBLE_EQ(-10, M(0, 0));
    EXPECT_DOUBLE_EQ( 30, M(0, 1));
    EXPECT_DOUBLE_EQ(  4, M(0, 2));
    EXPECT_DOUBLE_EQ( 30, M(1, 0));
    EXPECT_DOUBLE_EQ( 10, M(1, 1));
    EXPECT_DOUBLE_EQ( 20, M(1, 2));
    EXPECT_DOUBLE_EQ(  4, M(2, 0));
    EXPECT_DOUBLE_EQ(-20, M(2, 1));
    EXPECT_DOUBLE_EQ( 10, M(2, 2));
}

TEST_F(QuaternionTest, Check_ddcm_dqy_tr)
{
    auto M = q.ddcm_dqy_tr();
    EXPECT_DOUBLE_EQ(-30, M(0, 0));
    EXPECT_DOUBLE_EQ(-10, M(0, 1));
    EXPECT_DOUBLE_EQ(-20, M(0, 2));
    EXPECT_DOUBLE_EQ(-10, M(1, 0));
    EXPECT_DOUBLE_EQ( 30, M(1, 1));
    EXPECT_DOUBLE_EQ(  4, M(1, 2));
    EXPECT_DOUBLE_EQ( 20, M(2, 0));
    EXPECT_DOUBLE_EQ(  4, M(2, 1));
    EXPECT_DOUBLE_EQ(-30, M(2, 2));
}

TEST_F(QuaternionTest, Check_ddcm_dqz_tr)
{
    auto M = q.ddcm_dqz_tr();
    EXPECT_DOUBLE_EQ(-4,  M(0, 0));
    EXPECT_DOUBLE_EQ( 20, M(0, 1));
    EXPECT_DOUBLE_EQ(-10, M(0, 2));
    EXPECT_DOUBLE_EQ(-20, M(1, 0));
    EXPECT_DOUBLE_EQ(-4,  M(1, 1));
    EXPECT_DOUBLE_EQ( 30, M(1, 2));
    EXPECT_DOUBLE_EQ(-10, M(2, 0));
    EXPECT_DOUBLE_EQ( 30, M(2, 1));
    EXPECT_DOUBLE_EQ( 4,  M(2, 2));
}

TEST_F(QuaternionTest, IsQuatProdCorrect)
{
    auto res = static_cast<vector_form>(q * p);

    EXPECT_DOUBLE_EQ(-406, res[0]);
    EXPECT_DOUBLE_EQ( 113, res[1]);
    EXPECT_DOUBLE_EQ(-711, res[2]);
    EXPECT_DOUBLE_EQ(-320, res[3]);

    auto res_2 = static_cast<vector_form>(q *= p);

    EXPECT_DOUBLE_EQ(res_2[0], res[0]);
    EXPECT_DOUBLE_EQ(res_2[1], res[1]);
    EXPECT_DOUBLE_EQ(res_2[2], res[2]);
    EXPECT_DOUBLE_EQ(res_2[3], res[3]);
}

TEST_F(QuaternionTest, IsScalarProdCorrect)
{
    auto res = static_cast<vector_form>(q * 3);
    auto res_2 = static_cast<vector_form>(3 * q);
    auto res_3 = static_cast<vector_form>(q *= 3);

    EXPECT_DOUBLE_EQ( 30, res[0]);
    EXPECT_DOUBLE_EQ(-15, res[1]);
    EXPECT_DOUBLE_EQ( 45, res[2]);
    EXPECT_DOUBLE_EQ(  6, res[3]);

    EXPECT_DOUBLE_EQ(res_2[0], res[0]);
    EXPECT_DOUBLE_EQ(res_2[1], res[1]);
    EXPECT_DOUBLE_EQ(res_2[2], res[2]);
    EXPECT_DOUBLE_EQ(res_2[3], res[3]);

    EXPECT_DOUBLE_EQ(res_3[0], res[0]);
    EXPECT_DOUBLE_EQ(res_3[1], res[1]);
    EXPECT_DOUBLE_EQ(res_3[2], res[2]);
    EXPECT_DOUBLE_EQ(res_3[3], res[3]);
}

TEST_F(QuaternionTest, IsQuatSumCorrect)
{
    auto res = static_cast<vector_form>(q + p);

    EXPECT_DOUBLE_EQ(-35, res[0]);
    EXPECT_DOUBLE_EQ(  2, res[1]);
    EXPECT_DOUBLE_EQ( 16, res[2]);
    EXPECT_DOUBLE_EQ(-10, res[3]);

    auto res_2 = static_cast<vector_form>(q += p);

    EXPECT_DOUBLE_EQ(res_2[0], res[0]);
    EXPECT_DOUBLE_EQ(res_2[1], res[1]);
    EXPECT_DOUBLE_EQ(res_2[2], res[2]);
    EXPECT_DOUBLE_EQ(res_2[3], res[3]);
}

TEST_F(QuaternionTest, IsRotatorsCorrect)
{
    double angle = 1.4;
    double c = 0.7648;
    double s = 0.6442;

    auto xr = static_cast<vector_form>(x_rotator(angle));
    auto yr = static_cast<vector_form>(y_rotator(angle));
    auto zr = static_cast<vector_form>(z_rotator(angle));

    EXPECT_NEAR(c, xr[0], 1e-4);
    EXPECT_NEAR(c, yr[0], 1e-4);
    EXPECT_NEAR(c, zr[0], 1e-4);

    EXPECT_NEAR(s, xr[1], 1e-4);
    EXPECT_NEAR(s, yr[2], 1e-4);
    EXPECT_NEAR(s, zr[3], 1e-4);

    EXPECT_DOUBLE_EQ(0.0, xr[2]);
    EXPECT_DOUBLE_EQ(0.0, xr[3]);
    EXPECT_DOUBLE_EQ(0.0, yr[1]);
    EXPECT_DOUBLE_EQ(0.0, yr[3]);
    EXPECT_DOUBLE_EQ(0.0, zr[1]);
    EXPECT_DOUBLE_EQ(0.0, zr[2]);
}

TEST_F(QuaternionTest, IsSkewCorrect)
{
    auto V = skew_symmetric(q.vector_part());

    EXPECT_DOUBLE_EQ(0,   V(0, 0));
    EXPECT_DOUBLE_EQ(5,   V(0, 1));
    EXPECT_DOUBLE_EQ(-15, V(0, 2));
    EXPECT_DOUBLE_EQ(-2,  V(0, 3));
    EXPECT_DOUBLE_EQ(-5,  V(1, 0));
    EXPECT_DOUBLE_EQ(0,   V(1, 1));
    EXPECT_DOUBLE_EQ(2,   V(1, 2));
    EXPECT_DOUBLE_EQ(-15, V(1, 3));
    EXPECT_DOUBLE_EQ(15,  V(2, 0));
    EXPECT_DOUBLE_EQ(-2,  V(2, 1));
    EXPECT_DOUBLE_EQ(0,   V(2, 2));
    EXPECT_DOUBLE_EQ(-5,  V(2, 3));
    EXPECT_DOUBLE_EQ(2,   V(3, 0));
    EXPECT_DOUBLE_EQ(15,  V(3, 1));
    EXPECT_DOUBLE_EQ(5,   V(3, 2));
    EXPECT_DOUBLE_EQ(0,   V(3, 3));
}

TEST_F(QuaternionTest, IsLerpCorrect)
{
    auto res = static_cast<vector_form>(lerp(q, p, 0.6));

    EXPECT_NEAR(-0.9249, res[0], 1e-4);
    EXPECT_NEAR(0.0885,  res[1], 1e-4);
    EXPECT_NEAR(0.2654,  res[2], 1e-4);
    EXPECT_NEAR(-0.2574, res[3], 1e-4);
}

TEST_F(QuaternionTest, IsSlerpCorrect)
{
    // TODO implement failing test
    ASSERT_TRUE(true);
}
