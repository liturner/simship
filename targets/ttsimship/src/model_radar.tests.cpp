#include <gtest/gtest.h>

#include <Eigen/Core>

#include "TT/model_radar.h"

TEST(Radar, DefaultConstructor) {
  tt::simship::OwnshipChannel ownshipChannel;
  tt::simship::EnvironmentChannel environmentChannel;
  tt::simship::RadarModel radar(ownshipChannel, environmentChannel);
  ASSERT_TRUE(radar.load());
  ASSERT_TRUE(radar.init());
  ASSERT_TRUE(radar.reinit());
  ASSERT_TRUE(radar.run());
  ASSERT_TRUE(radar.hold());
  ASSERT_TRUE(radar.unload());
}

TEST(Radar, EnemyInView) {
  tt::rpr_fom::PhysicalEntity anEnemy;
  anEnemy.Spatial.SpatialRVW.WorldLocation.X = 10000;
  anEnemy.Spatial.SpatialRVW.WorldLocation.Y = 2;
  anEnemy.Spatial.SpatialRVW.WorldLocation.Z = 2;

  tt::simship::OwnshipChannel ownshipChannel;
  tt::simship::EnvironmentChannel environmentChannel;
  environmentChannel.physicalEntities.getWriteHandle()->push_back(anEnemy);
  tt::simship::RadarModel radar(ownshipChannel, environmentChannel);
  ASSERT_TRUE(radar.load());
  ASSERT_TRUE(radar.init());
  ASSERT_TRUE(radar.reinit());
  ASSERT_TRUE(radar.run());

  ASSERT_EQ(1, radarChannel::echos.size());
  auto echo = radarChannel::echos.front();
  radarChannel::echos.pop();

  ASSERT_TRUE(radar.hold());
  ASSERT_TRUE(radar.unload());
}

TEST(Radar, EnemyOutOfRange) {
  tt::rpr_fom::PhysicalEntity anEnemy;
  anEnemy.Spatial.SpatialRVW.WorldLocation.X = 20000;

  tt::simship::OwnshipChannel ownshipChannel;
  tt::simship::EnvironmentChannel environmentChannel;
  environmentChannel.physicalEntities.getWriteHandle()->push_back(anEnemy);
  tt::simship::RadarModel radar(ownshipChannel, environmentChannel);
  ASSERT_TRUE(radar.load());
  ASSERT_TRUE(radar.init());
  ASSERT_TRUE(radar.reinit());
  ASSERT_TRUE(radar.run());

  ASSERT_EQ(0, radarChannel::echos.size());

  ASSERT_TRUE(radar.hold());
  ASSERT_TRUE(radar.unload());
}
