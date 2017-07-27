#include <unistd.h>
#include "mrptpf.h"
#include <gtest/gtest.h>

#include <math.h>


TEST(PraticleFilter, ParticlesSize){
    CMetricMapBuilderRBPF::TConstructionOptions  newOptions;
    newOptions.PF_options.sampleSize = 500;
    mrptPF pf(newOptions);
    CPose3DPDFParticles p = pf.getCurrentPDF();
    EXPECT_EQ(500, p.m_particles.size());
}

TEST(PraticleFilter, ParticlePose){
    CMetricMapBuilderRBPF::TConstructionOptions  newOptions;
    newOptions.PF_options.sampleSize = 100;
    mrptPF pf(newOptions);
    CPose3DPDFParticles p = pf.getCurrentPDF();
    EXPECT_NEAR(p.getMeanVal().x(), 0, 0.1);
}


int main(int argc, char **argv){
    cout << "Running unit tests for particle filter" << endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    return 0;
}
