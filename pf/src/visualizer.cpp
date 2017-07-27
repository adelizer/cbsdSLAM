#include "visualizer.h"

Visualizer::Visualizer(){
    config();
}

Visualizer::~Visualizer(){}

void Visualizer::config(){
    winSLAM2d.setPos(1000,10);
    winSLAM2d.resize(640, 480);
    winSLAM2d.axis(-3, 3, -3, 3);

}

void Visualizer::showParticles(){
    winSLAM2d.clf(); // clear figure, prepare for next draw

    vector<float> plotX, plotY;
    plotX.push_back(pf->currentPose.x());
    plotX.push_back(pf->currentPose.x() + 0.3*cos(pf->currentPose.yaw()));
    plotY.push_back(pf->currentPose.y());
    plotY.push_back(pf->currentPose.y() + 0.3*sin(pf->currentPose.yaw()));

    for(int i = 0; i < pf->currentPDF.particlesCount(); i++){
        // get current particle (i) pose
        CPose3D particlePose = pf->currentPDF.getParticlePose(i);
        // plot the location as a dot
        winSLAM2d.plot( vector<float>(1, particlePose.x()), vector<float>(1,particlePose.y()),"r.2","particls");
        winSLAM2d.hold_on(); // hold on for other particles
    }
    CMatrixDouble COV;
    pf->currentPDF.getCovariance(COV);
    CMatrixDouble22 COV2 = COV.block(0,0,2,2);
    winSLAM2d.plotEllipse(pf->currentPose.x(), pf->currentPose.y(), COV2, 3, "b-2", "Covariance");
    winSLAM2d.hold_on();
    winSLAM2d.plot( plotX, plotY,"g-4","angle");
    winSLAM2d.hold_on();


}

void Visualizer::showGrid(){
    if(!pf->gridMapAvailable)
        return;
    maps::COccupancyGridMap2DPtr grid = pf->mostLikMap->getMapByClass<maps::COccupancyGridMap2D>();
    CImage	img;
    grid->getAsImage(img);
    frame.showImage(img);

}

void Visualizer::showPointsMap(){
    if(!pf->pointsMapAvailable)
        return;
    vector<float>   map1_xs, map1_ys, map1_zs;
    CSimplePointsMapPtr map_1 = pf->mostLikMap->getMapByClass<maps::CSimplePointsMap>();
    map_1->getAllPoints(map1_xs,map1_ys,map1_zs);
    winSLAM2d.plot( map1_xs, map1_ys, "b.3", "map1");
    winSLAM2d.hold_on();
}

void Visualizer::showLandmarks(){
    if(!pf->landmarksMapAvailable)
        return;
    if(pf->mostLikMap->m_landmarksMap.present()){
        CLandmarksMapPtr map = pf->mostLikMap->getMapByClass<maps::CLandmarksMap>();
        for(int i = 0; i < map->landmarks.size(); i++){
            CLandmark *l = pf->currentMap->m_landmarksMap->landmarks.get(i);
            winSLAM2d.plot( vector<float>(1,l->pose_mean.x ), vector<float>(1,l->pose_mean.y),"m.9","land");
            CMatrixDouble22 cov;
            cov(0,0) = l->pose_cov_11; cov(1,1) = l->pose_cov_22;
            winSLAM2d.plotEllipse(l->pose_mean.x,l->pose_mean.y, cov, 3, "k-1", "Covariance");
            winSLAM2d.hold_on();
        }
    }

}

void Visualizer::show3D(){
    COpenGLScenePtr scene;
    scene = COpenGLScene::Create(); // create a scene !

    // The ground:
    mrpt::opengl::CGridPlaneXYPtr groundPlane = mrpt::opengl::CGridPlaneXY::Create(-200,200,-200,200,0,5);
    groundPlane->setColor(0.4,0.4,0.4);
    scene->insert( groundPlane );

    // The camera pointing to the current robot pose:
    mrpt::opengl::CCameraPtr objCam = mrpt::opengl::CCamera::Create();
    CPose3D robotPose;
    pf->currentPDF.getMean(robotPose);

    objCam->setPointingAt(robotPose);
    objCam->setAzimuthDegrees(-80);
    objCam->setElevationDegrees(30);


    // Draw the robot particles:
    size_t M = pf->mapPDF.particlesCount();
    CSetOfLinesPtr objLines = CSetOfLines::Create();
    objLines->setColor(0,1,0);
    for (size_t i = 0; i < M; i++){
        deque<TPose3D> path;
        pf->mapPDF.getPath(i,path);
        int n = path.size();
        objLines->appendLine(path[n-1].x, path[n-1].y, path[n-1].z, path[n-2].x, path[n-2].y, path[n-2].z+0.001);
        scene->insert(objLines);
    }

    CMatrixDouble COV;
    pf->currentPDF.getCovariance(COV);
    opengl::CEllipsoidPtr objEllip = opengl::CEllipsoid::Create();
    objEllip->setLocation(pf->currentPose.x(), pf->currentPose.y(), pf->currentPose.z() + 0.001 );
    objEllip->setCovMatrix(COV, COV(2,2)==0 ? 2:3 );
    objEllip->setColor(0,0,1);
    objEllip->enableDrawSolid3D(false);
    scene->insert( objEllip );

    COpenGLScenePtr &scenePtr = win3D->get3DSceneAndLock();
    scenePtr = scene;
    win3D->unlockAccess3DScene();
    win3D->forceRepaint();
}


bool Visualizer::checkKill(){
    bool kill;
    if (os::kbhit()){
        char c = os::getch();
        if (c==27) // if ESC
            kill = true;
    }
    if(!winSLAM2d.isOpen() || !frame.isOpen()){ // if the window is closed
        cout << " Terminating " << endl;
        kill = true;
    }
    pf->kill = kill;
    return kill;
}


void Visualizer::printInfo(){

    cout << "Particles number " << pf->currentPDF.m_particles.size()  <<
            ", X: " << pf->currentPose.x() << ", Y: " << pf->currentPose.y() <<
            ", Theta:" << pf->currentPose.yaw() << endl;


}

void Visualizer::printRandom(){


    CBeaconMapPtr beaconMap = pf->mostLikMap->getMapByClass<maps::CBeaconMap>();
    COccupancyGridMap2DPtr gridMap = pf->mostLikMap->getMapByClass<maps::COccupancyGridMap2D>();

    if(beaconMap->size() > 0){
        CBeacon bec = beaconMap->get(0);
        cout << "Now we have a new beacon " << bec.getMeanVal().x() << endl;
    }
    cout << pf->mostLikMap->maps.size() << endl;
    cout << "Landmarks in the map " <<   pf->mapPDF.getCurrentMostLikelyMetricMap()->m_landmarksMap->size()<< endl;
    cout << "Beacons in the map " <<   pf->mapPDF.getCurrentMostLikelyMetricMap()->m_beaconMap->size() << endl;
    cout << "points in the map " <<   pf->mapPDF.getCurrentMostLikelyMetricMap()->m_pointsMaps.size() << endl;
    cout << "current grid map " << pf->mapPDF.getCurrentMostLikelyMetricMap()->m_gridMaps.size() << endl;
    cout << "current map size " <<  pf->getCurrentlyBuiltMapSize() << ", " << pf->step << endl;
}
