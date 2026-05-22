#include <gtest/gtest.h>
#include <cmath>
#include "../include/algorithms/planar_imu_integrator.hpp"

using algorithms::PlanarImuIntegrator;

constexpr float ERROR = 0.001f;

// --- Test 1: Po vytvorení má yaw aj offset hodnotu 0 ---
TEST(PlanarImuIntegratorTest, InitialValuesAreZero) {
    PlanarImuIntegrator imu;
    EXPECT_NEAR(imu.getYaw(), 0.0f, ERROR);
    EXPECT_NEAR(imu.getOffset(), 0.0f, ERROR);
}

// --- Test 2: Integrácia konštantnej rýchlosti 1 rad/s po dobu 1 sekundy → 1 rad ---
TEST(PlanarImuIntegratorTest, IntegrateConstantRate) {
    PlanarImuIntegrator imu;
    // 100 krokov po 10 ms = 1 sekunda, pri rýchlosti 1 rad/s → 1 rad
    for (int i = 0; i < 100; ++i) {
        imu.update(1.0f, 0.01);
    }
    EXPECT_NEAR(imu.getYaw(), 1.0f, ERROR);
}

// --- Test 3: Kalibrácia odhadne bias správne ---
TEST(PlanarImuIntegratorTest, CalibrationComputesMean) {
    PlanarImuIntegrator imu;
    // 5 vzoriek s priemerom 0.1
    std::vector<float> samples = {0.05f, 0.1f, 0.15f, 0.08f, 0.12f};
    imu.setCalibration(samples);
    float expected_mean = (0.05f + 0.1f + 0.15f + 0.08f + 0.12f) / 5.0f;
    EXPECT_NEAR(imu.getOffset(), expected_mean, ERROR);
}

// --- Test 4: Po kalibrácii sa bias odčítava – konštantný drift = 0 ---
TEST(PlanarImuIntegratorTest, BiasIsSubtracted) {
    PlanarImuIntegrator imu;
    // Simulujeme, že robot stojí a gyro zakaždým vraj 0.1 rad/s (to je bias)
    std::vector<float> calibration_samples(100, 0.1f);
    imu.setCalibration(calibration_samples);

    // Teraz integrujeme 1 sekundu "pokoja" (gyro_z je stále 0.1)
    for (int i = 0; i < 100; ++i) {
        imu.update(0.1f, 0.01);
    }
    // Bias sa odčítal, takže yaw by mal ostať ~0
    EXPECT_NEAR(imu.getYaw(), 0.0f, ERROR);
}

// --- Test 5: Kombinácia – bias + skutočné otáčanie ---
TEST(PlanarImuIntegratorTest, BiasAndRealRotation) {
    PlanarImuIntegrator imu;
    // Bias = 0.1 rad/s
    std::vector<float> calibration_samples(100, 0.1f);
    imu.setCalibration(calibration_samples);

    // Robot sa teraz otáča skutočnou rýchlosťou 0.5 rad/s
    // Ale gyro čítalo by 0.5 + 0.1 (bias) = 0.6 rad/s
    for (int i = 0; i < 100; ++i) {
        imu.update(0.6f, 0.01);
    }
    // Očakávaný výsledok: 0.5 rad/s × 1 s = 0.5 rad
    EXPECT_NEAR(imu.getYaw(), 0.5f, ERROR);
}

// --- Test 6: reset() vyčistí všetko ---
TEST(PlanarImuIntegratorTest, ResetClearsEverything) {
    PlanarImuIntegrator imu;
    imu.setCalibration({0.1f, 0.1f, 0.1f});
    imu.update(1.0f, 0.5);
    EXPECT_GT(imu.getYaw(), 0.0f);  // yaw je nenulový

    imu.reset();
    EXPECT_NEAR(imu.getYaw(), 0.0f, ERROR);
    EXPECT_NEAR(imu.getOffset(), 0.0f, ERROR);
}

// --- Test 7: resetYaw() zachová offset ---
TEST(PlanarImuIntegratorTest, ResetYawKeepsOffset) {
    PlanarImuIntegrator imu;
    imu.setCalibration({0.1f, 0.1f, 0.1f});
    imu.update(1.0f, 0.5);
    imu.resetYaw();
    EXPECT_NEAR(imu.getYaw(), 0.0f, ERROR);
    EXPECT_NEAR(imu.getOffset(), 0.1f, ERROR);  // offset ostal
}

// --- Test 8: Rotácia o 90° = π/2 ---
TEST(PlanarImuIntegratorTest, NinetyDegreeRotation) {
    PlanarImuIntegrator imu;
    // 1 rad/s počas π/2 sekundy → π/2 rad
    int steps = 157;  // π/2 × 100 ≈ 157 krokov × 10 ms
    for (int i = 0; i < steps; ++i) {
        imu.update(1.0f, 0.01);
    }
    EXPECT_NEAR(imu.getYaw(), M_PI_2, 0.01f);  // malá tolerancia kvôli diskretizácii
}

// --- Test 9: Prázdne vzorky pri kalibrácii nespôsobia crash ---
TEST(PlanarImuIntegratorTest, EmptyCalibrationIsSafe) {
    PlanarImuIntegrator imu;
    std::vector<float> empty_samples;
    imu.setCalibration(empty_samples);
    EXPECT_NEAR(imu.getOffset(), 0.0f, ERROR);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}