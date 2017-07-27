#include "mrptpf.h"

mrptPF::mrptPF(CMetricMapBuilderRBPF::TConstructionOptions newOptions) : CMetricMapBuilderRBPF(newOptions){
    actions = CActionCollection::Create(); // create actions container
    observations = CSensoryFrame::Create(); // create observations container
    scanSensor = CObservation2DRangeScan::Create();

    newOptions.dumpToConsole(); // display all options

    pointsMapAvailable = false;
    landmarksMapAvailable = false;
    gridMapAvailable = false;


    for (std::deque<TMetricMapInitializerPtr>::iterator it=newOptions.mapsInitializers.begin(); it!=newOptions.mapsInitializers.end();++it){
        TMetricMapInitializerPtr temp = (*it);
        const mrpt::utils::TRuntimeClassIdPtr type = temp->getMetricMapClassType();
        if(!strcmp(type->className, "CLandmarksMap"))
            landmarksMapAvailable = true;

        if(!strcmp(type->className, "CSimplePointsMap"))
            pointsMapAvailable = true;

        if(!strcmp(type->className, "COccupancyGridMap2D")){
            gridMapAvailable = true;
            cout << "GRID map identified" << endl;
        }
    }


    kill = false; // do not kill, yet
    randomGenerator.randomize(); // initialize the random generator
    step = 0; // process counter
    config(); // config parameters
}

// destrcutor
mrptPF::~mrptPF(){
    cout << "\n Saving map  to file " << endl;
    try {
        std::string sOutMap = "mrpt_rbpfslam_";
        mrpt::system::TTimeParts parts;
        mrpt::system::timestampToParts(now(), parts, true);
        sOutMap += format("%04u-%02u-%02u_%02uh%02um%02us",
                          (unsigned int)parts.year,
                          (unsigned int)parts.month,
                          (unsigned int)parts.day,
                          (unsigned int)parts.hour,
                          (unsigned int)parts.minute,
                          (unsigned int)parts.second );
        sOutMap += ".simplemap";
        mapPDF.saveCurrentPathEstimationToTextFile("./../logging/sessions/pathEstimation");
        //            mostLikMap->saveMetricMapRepresntationToFile("/results/map.txt");
        //            mostLikMap->m_landmarksMap->saveToMATLABScript2D("landmarks", "b", 2.0);

        sOutMap = mrpt::system::fileNameStripInvalidChars( sOutMap );
        //            saveCurrentMapToFile(sOutMap);
        cout << "\n Current map saved to file " << endl;
    } catch (std::exception &e) {
        //            ROS_ERROR("Exception: %s",e.what());
    }

}

void mrptPF::config(){
    options.enableMapUpdating = true;
    options.alwaysInsertByClass.insert(CLASS_ID(CObservationBeaconRanges)); // always insert observations of class type
    options.debugForceInsertion = true; // update map even with no motion -- same effect as always insert
    m_PF_options.adaptiveSampleSize = false;
    m_PF_options.PF_algorithm = CParticleFilter::TParticleFilterAlgorithm::pfStandardProposal;
    m_PF_options.resamplingMethod = CParticleFilter::TParticleResamplingAlgorithm::prSystematic; // low variance resampling
    clear(); // clear all elements in the map

    // helper variables to test what are the interfaces
    CProbabilityParticle<CRBPFParticleData> data;
    //    data.log_w
    //    data.d->mapTillNow.m_pointsMaps
    TPose3D t;

    for(int i = 0; i < mapPDF.m_particles.size(); i++){ // loop and random generate particles
        // get current particle (i) pose and randomly distribute in the covariance
        mapPDF.m_particles[i].d->robotPath[0].x = randomGenerator.drawGaussian1D(0.0f, 0.1f);
        mapPDF.m_particles[i].d->robotPath[0].y = randomGenerator.drawGaussian1D(0.0f, 0.1f);
        //        mapPDF.m_particles[i].d->robotPath[0].yaw =  randomGenerator.drawGaussian1D(0.0f, 0.01f);

    }

    // Motion model noise parameters
    odo_opts.modelSelection = CActionRobotMovement2D::mmGaussian;
    odo_opts.gaussianModel.a1  = 0.01; //0.01f;
    odo_opts.gaussianModel.a2  = 0.001; //RAD2DEG( 0.0001f );
    odo_opts.gaussianModel.a3  = DEG2RAD(5); //DEG2RAD( 0.1f );
    odo_opts.gaussianModel.a4  = DEG2RAD(0.05); //0.05f;
    odo_opts.gaussianModel.minStdXY  = 0.005;
    odo_opts.gaussianModel.minStdPHI =  DEG2RAD(0.2);

    scanSensor->aperture = 1.56466+1.5708;
    scanSensor->rightToLeft = true;
    scanSensor->beamAperture = 0.00613592;
    scanSensor->maxRange = 5.6;
    scanSensor->setSensorPose(CPose3D(0.3, 0.0, 0.0, 0.0, 0.0,0.0));
    scanSensor->stdError = 0.01;

    observationNoise.zeros();
    observationNoise(0,0) = 0.1f;
    observationNoise(1,1) = DEG2RAD(5.0f);

    currentMap = this->getCurrentlyBuiltMetricMap();
    currentPDFptr = getCurrentPoseEstimation();
    if ( IS_CLASS( currentPDFptr, CPose3DPDFParticles ) ){
        CPose3DPDFParticlesPtr pp= CPose3DPDFParticlesPtr(currentPDFptr);
        currentPDF = *pp;
    }
    currentPDF.getMean(currentPose);


}

void mrptPF::setInput(double x, double y, double theta){
    actions->clear(); // clear previous actions
    CActionRobotMovement2D action;
    CPose2D poseIncr(x, y, theta);
    action.computeFromOdometry(poseIncr, odo_opts);
    action.timestamp = mrpt::system::now();

    actions->insert(action);

}


void mrptPF::setObservations(vector<double> &x, vector<double> &y, vector<double> &z, vector<int> &ids){
    if(!landmarksMapAvailable){
        cout <<  "skipping range-bearing observation.. map unavailable" << endl;
        return;
    }
    observations->clear(); // clear previous observations
    assert(x.size() == z.size() && "size of x is not equal to size of z");
    assert(x.size() == ids.size() && "size of x is not equal to size of ids");

    for(int i = 0; i < z.size(); i++){ // loop-i all measurements
        if(z[i] > 3.5) // range at which angle can be determined accurately
            continue;

        double range = z[i];
        double bearing = -1 * atan2(x[i], z[i]);

        bool skip = false;
        for(int j = 0; j < observedLandmarks.size(); j++){ // loop-j all observed landmarks
            if(ids[i] == observedLandmarks[j]){
                // go and update, resample, etc.
                for(int k = 0; k < mapPDF.m_particles.size(); k++){ // k-loop all particles
                    updateParticle(range, bearing, ids[i], k);
                }
                skip = true;
            }
        }
        if(!skip){ // add a new feature to the map of each particle
            observedLandmarks.push_back(ids[i]);
            int pathL = mapPDF.m_particles[0].d->robotPath.size();

            for(int k = 0; k < mapPDF.m_particles.size(); k++){ // k-loop all particles
                CLandmark l;
                l.ID = ids[i];
                double rx,ry,rtheta, s,c;
                rx = mapPDF.m_particles[k].d->robotPath[pathL-1].x;
                ry = mapPDF.m_particles[k].d->robotPath[pathL-1].y;
                rtheta = mapPDF.m_particles[k].d->robotPath[pathL-1].yaw;
                c = cos(rtheta + bearing);
                s = sin(rtheta + bearing);

                double lx, ly;
                lx = rx + z[i] * c;
                ly = ry + z[i] * s;
                l.setPose(CPointPDFGaussian(CPoint3D(lx, ly, 0)));
                CMatrixDouble22 Gz;
                Gz(0,0) = c;
                Gz(0,1) = -range * s;
                Gz(1,0) = s;
                Gz(1,1) = range * c;
                CMatrixDouble22 featureCov;
                featureCov = Gz * observationNoise * Gz.transpose();
                l.pose_cov_11 = featureCov(0,0);
                l.pose_cov_22 = featureCov(1,1);
                mapPDF.m_particles[k].d->mapTillNow.m_landmarksMap->landmarks.push_back(l);
            } // end of k-loop
        } // end of if() add new feature
    } // end of i-loop for each measurement
} // end of setObservations()

void mrptPF::setObservations(vector<float> &ranges, vector<char> &valid){
    if(!pointsMapAvailable && !gridMapAvailable){
        cout <<  "skipping scan observation.. map unavailable" << endl;
        return;
    }

    observations->clear(); // clear previous observations
    float* r = &ranges[0];
    char* v = &valid[0];
    scanSize = ranges.size();
    // maybe change sensor pose ?
    scanSensor->loadFromVectors(scanSize, r, v);
    observations->insert(scanSensor);

}



void mrptPF::doProcess(){
    this->enableMapUpdating(true);
    processActionObservation(*actions, *observations);
    mostLikMap = mapPDF.getCurrentMostLikelyMetricMap();
    currentMap = this->getCurrentlyBuiltMetricMap();
    currentPDFptr = getCurrentPoseEstimation();
    if ( IS_CLASS( currentPDFptr, CPose3DPDFParticles ) ){
        CPose3DPDFParticlesPtr pp= CPose3DPDFParticlesPtr(currentPDFptr);
        currentPDF = *pp;
    }
    currentPDF.getMean(currentPose);
    actions->clear();
    observations->clear();
    step++;

    //    testBeaconMap();
}


void inline mrptPF::wrapAngle(double &angle){
    double temp = angle;
    if(angle>M_PI)
        angle=temp-2*M_PI;
    else if(angle<-M_PI)
        angle = temp+2*M_PI;

}

int mrptPF::getLandmarkOrder(int id, int particleID){
    int n = mapPDF.m_particles[particleID].d->mapTillNow.m_landmarksMap->landmarks.size();
    int result = -1;
    for(int i = 0; i < n; i++){
        CLandmark *l = mapPDF.m_particles[i].d->mapTillNow.m_landmarksMap->landmarks.get(i);
        if(l->ID == id)
            result = i;
    }
    return result;
}

void mrptPF::updateParticle(double range, double bearing, int id, int i){
    // get the re-observed landmark for particle i
    int landmarkOrder = getLandmarkOrder(id,i);
    CLandmark *l = mapPDF.m_particles[i].d->mapTillNow.m_landmarksMap->landmarks.get(landmarkOrder);
    int pathL = mapPDF.m_particles[i].d->robotPath.size();
    double rx,ry,rtheta, s,c;
    rx = mapPDF.m_particles[i].d->robotPath[pathL-1].x;
    ry = mapPDF.m_particles[i].d->robotPath[pathL-1].y;
    rtheta = mapPDF.m_particles[i].d->robotPath[pathL-1].yaw;

    double lx = l->pose_mean.x;
    double ly = l->pose_mean.y;
    double dx = l->pose_mean.x - rx;
    double dy = l->pose_mean.y - ry;
    double d2 = dx*dx + dy*dy;
    double d = sqrt(d2);

    double predictedRange = d;
    double predictedBearing = atan2(dy,dx) - rtheta;
    wrapAngle(predictedBearing);

    // update the location of the landmark and the covariance
    Vector2d v(range - predictedRange, bearing - predictedBearing), resultX;
    CMatrixDouble22 hf;
    hf(0,0) = dx/d; hf(0,1) = dy/d;
    hf(1,0) = -dy/d2; hf(1,1) = dx/d2;
    CPoint3D tempPoint;
    CMatrixDouble cov3, cov2, PHt, str, S, w, w1,  resultP;
    l->getPose(tempPoint, cov3);
    cov2 = cov3.block(0,0,2,2);
    PHt = cov2 * hf.transpose();
    S = hf * PHt + observationNoise;
    str = S.transpose();
    S = (S + str);
    S = S * 0.5;
    MatrixXd L( S.llt().matrixL() ), SCholInv;
    SCholInv = L.inverse();
    w1 = PHt *  L.inverse();
    w = w1 * SCholInv.transpose();
    resultX = w*v;
    resultP = cov2 - w1*w1.transpose();
    //    mapPDF.m_particles[i].d->mapTillNow.m_landmarksMap->landmarks[landmarkOrder].setPose(CPointPDFGaussian(CPoint3D(lx + result(0), ly + result(1))));
    CMatrixDouble33 fUpdate;
    fUpdate.zeros();
    fUpdate(0,0) = resultP(0,0); fUpdate(0,1) = resultP(0,1);
    fUpdate(1,0) = resultP(1,0); fUpdate(1,1) = resultP(1,1);
    fUpdate(2,2) = 0.001;
    l->setPose(CPointPDFGaussian(CPoint3D(lx + resultX(0), ly + resultX(1), 0), fUpdate));

    if(i == 0){
        //        cout << "How much the location of the landmark is updated \n " << resultX  << endl;
        //        cout << "=====" << endl;
    }

    // update the weight of the particle
    mapPDF.m_particles[i].log_w +=
            log( math::normalPDF( predictedRange-range, 0, observationNoise(0,0) ) ) +
            log( math::normalPDF( math::wrapToPi( predictedBearing-bearing), 0, observationNoise(1,1) ) );
}


CPose3DPDFParticles mrptPF::getCurrentPDF() const{
    return currentPDF;
}

void mrptPF::setCurrentPDF(const CPose3DPDFParticles &value){
    currentPDF = value;
}

void mrptPF::getPose(double &x, double &y, double &theta){
    x = currentPose.x();
    y = currentPose.y();
    theta = currentPose.yaw();
}
