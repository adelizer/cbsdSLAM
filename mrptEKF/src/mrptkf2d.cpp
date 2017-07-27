#include "mrptkf2d.h"

mrptKF2d::mrptKF2d(){ // constructor

    // Create the container for the input commands and the observations
    actions = CActionCollection::Create();
    observations = CSensoryFrame::Create();

    // don't kill yet
    kill = false;

    // go config
    config();
}

mrptKF2d::~mrptKF2d(){ // destructor
    // Saving the path into a txt file
    mrpt::system::vectorToTextFile(pathx, "pathx.txt");
    mrpt::system::vectorToTextFile(pathy, "pathy.txt");
}

void mrptKF2d::config(){
    configFileName = "/home/adel/workspace/slam/primitives/mrptEKF/config/ekf.ini";
    CConfigFile		iniFile( configFileName );

    // parse the input from the configuration file with default values
    m_xkk[0] = iniFile.read_double("EKF", "START_POSE_X", 0.0);
    m_xkk[1] = iniFile.read_double("EKF", "START_POSE_Y", 0.0);
    m_xkk[2] = iniFile.read_double("EKF", "START_POSE_THETA", 0.0);

    m_pkk(0,0) = iniFile.read_double("EKF", "INIITAL_COV_X", 0.01);
    m_pkk(1,1)= iniFile.read_double("EKF", "INIITAL_COV_Y", 0.01);
    m_pkk(2,2)= iniFile.read_double("EKF", "INIITAL_COV_THETA", 0.001);

    input.resize(3);
    input.zeros();

    alpha.resize(4);
    alpha(0) = iniFile.read_double("EKF", "ALPHA_1", 0.5);
    alpha(1) = iniFile.read_double("EKF", "ALPHA_2", 0.00001);
    alpha(2) = iniFile.read_double("EKF", "ALPHA_3", 0.00001);
    alpha(3) = iniFile.read_double("EKF", "ALPHA_4", 0.01);

    deltaTime = iniFile.read_double("EKF", "DELTA_TIME", 0.01);

    rangeNoise = iniFile.read_double("EKF", "RANGE_NOISE", 0.1);
    bearingNoise = iniFile.read_double("EKF", "BEARING_NOISE", 0.03);

    minSensorRange = iniFile.read_double("EKF", "SENSOR_MIN_RANGE", 0.1);
    maxSensorRange = iniFile.read_double("EKF", "SENSOR_MAX_RANGE", 3.5);

    winSLAM2d.setPos(1000,10);
    winSLAM2d.resize(640, 480);
    winSLAM2d.axis(-3,3,-3,3);
    winSLAM2d.axis_equal();

    CActionRobotMovement2D actMov;
    actions->insert(actMov);
    CObservationBearingRangePtr sensor = CObservationBearingRange::Create();
    observations->insert(sensor);
}

void mrptKF2d::setInput(double x, double y, double theta){
    input[0] = x;
    input[1] = y;
    input[2] = theta;
}

void mrptKF2d::setObservations(vector<double> &x, vector<double> &y, vector<double> &z, vector<int> &ids){
    if((z.size() != ids.size()) || (x.size() != z.size()) ){
        cerr << "The size of the observation is not consistent" <<  endl;
        return;
    }
    observations = CSensoryFrame::Create();
    CObservationBearingRangePtr sensor = CObservationBearingRange::Create();
    observations->insert(sensor);

    for(int i = 0; i < z.size(); i++){ // for each observation
        if(z[i] < minSensorRange || z[i] > maxSensorRange)
            return;
        CObservationBearingRange::TMeasurement t;
        t.range = z[i];
        t.pitch = 0.0; // by default since 2D SLAM. Has to be set
        t.yaw = -1 * atan2(x[i], z[i]); // get the angle
        cout << "Landmark " << ids[i] << "   has angle of " << RAD2DEG(t.yaw) << " and range of " <<  z[i] << endl;
        t.landmarkID = ids[i];
        sensor->sensedData.push_back(t);
        observations->insert(sensor);
    }
}

void mrptKF2d::doProcess(){
    x = m_xkk[0];
    y = m_xkk[1];
    theta = m_xkk[2];
    v = input[0];
    w = input[2];
    processActionObservation(actions, observations);
    //    actions->clear();
    //    observations->clear(); // MRPT throws an exception when observation is cleared
    // work around to clear
    observations = CSensoryFrame::Create();
    CObservationBearingRangePtr sensor = CObservationBearingRange::Create();
    observations->insert(sensor);
    input.zeros();
}



void mrptKF2d::OnGetAction(CKalmanFilterCapable::KFArray_ACT &out_u) const{
    // override the function to return my input
    out_u[0] = input[0];
    out_u[1] = input[1];
    out_u[2] = input[2];

}

void mrptKF2d::OnTransitionModel(const CKalmanFilterCapable::KFArray_ACT &in_u, CKalmanFilterCapable::KFArray_VEH &inout_x, bool &out_skipPrediction) const{
    double tempTheta = theta;
    if( std::fabs(w) > 0.0001 ){
        inout_x[0] = x + (-v/w) * sin(theta) + (v/w) * sin(theta + deltaTime*w);
        inout_x[1] = y + (v/w) * cos(theta) - (v/w) * cos(theta + deltaTime*w);
        tempTheta = tempTheta + w*deltaTime;

    }else{
        inout_x[0] = x + v*cos(theta)*deltaTime;
        inout_x[1] = y + v*sin(theta)*deltaTime;
        tempTheta = tempTheta;
    }

    if(tempTheta>M_PI)
        tempTheta=tempTheta-2*M_PI;
    else if(tempTheta<-M_PI)
        tempTheta = tempTheta+2*M_PI;

    inout_x[2] = tempTheta;
    out_skipPrediction = false;
}


void mrptKF2d::OnTransitionJacobian(CKalmanFilterCapable::KFMatrix_VxV &out_F) const{
    out_F.unit();
    if(std::fabs(w) > 0.0001){
        out_F(0,2) = ((-v/w) * cos(theta)) + ((v/w) * cos(theta + w*deltaTime));
        out_F(1,2) = ((-v/w) * sin(theta)) + ((v/w) * sin(theta + w*deltaTime));
    }else{
        out_F(0,2) = -v*sin(theta)*deltaTime;
        out_F(1,2) = v*cos(theta)*deltaTime;
    }
}

void mrptKF2d::OnTransitionNoise(CKalmanFilterCapable::KFMatrix_VxV &out_Q) const{
    KFMatrix V(3,2);
    KFMatrix M(2,2);
    M(0,0) = (alpha(0) * v * v) + (alpha(1) * w * w);
    M(1,1) = (alpha(2) * v * v) + (alpha(3) * w * w);
    M(0,1) = 0.0;
    M(1,0) = 0.0;
    if(std::fabs(w) > 0.0001){
        V(0,0) = (-sin(theta) + sin(theta+w*deltaTime))/w;
        V(0,1) = (sin(theta) - sin(theta+w*deltaTime))*v/(w*w) + v*deltaTime*cos(theta+w*deltaTime)/w;
        V(1,0) = (cos(theta) - cos(theta+w*deltaTime))/w;
        V(1,1) = -1*((cos(theta) - cos(theta+w*deltaTime))*v/(w*w)) + v*deltaTime*sin(theta+w*deltaTime)/w;
        V(2,1) = deltaTime;
    }else{
        V(0,0) = cos(theta) * deltaTime;
        V(1,0) = sin(theta) * deltaTime;
        V(2,1) = deltaTime;
    }
    out_Q = ( V * M * V.transpose() );
}

void mrptKF2d::OnGetObservationNoise(KFMatrix_OxO &out_R) const{
//    out_R.ones();
    out_R(0,0) = rangeNoise;
    out_R(1,1) = bearingNoise;
}

void mrptKF2d::OnNormalizeStateVector(){
    // can be used to save the points, path, wrap angles, etc.
}


void mrptKF2d::visualize(){
    if(!winSLAM2d.isOpen()){
        kill = true;
        return;
    }

    if(pathx.empty()){
        pathx.push_back(m_xkk[0]);
        pathy.push_back(m_xkk[1]);
    }else{
        double diffx = pathx[pathx.size()-1] - m_xkk[0];
        double diffy = pathx[pathx.size()-1] - m_xkk[0];
        double dist = sqrt(pow(diffx,2) + pow(diffy,2));
        if(dist > 0.01){
            pathx.push_back(m_xkk[0]);
            pathy.push_back(m_xkk[1]);
        }
    }

    KFMatrix   circle(2,2);
    circle(0,0) = 0.005;
    circle(1,1) = 0.005;
    circle(0,1) = circle(1,0) = 0.0;

    KFMatrix   COVXY(2,2);
    COVXY(0,0) = m_pkk(0,0);
    COVXY(1,1) = m_pkk(1,1);
    COVXY(0,1) = COVXY(1,0) = m_pkk(0,1);

    winSLAM2d.clf();
    winSLAM2d.plot( vector<float>(1,m_xkk[0]), vector<float>(1,m_xkk[1]),"k.8","loc");
    winSLAM2d.hold_on();
    for(int i = 3; i < m_xkk.rows()-1; i+=2){
        winSLAM2d.plot( vector<float>(1,m_xkk[i]), vector<float>(1,m_xkk[i+1]),"r.9","land");
    }
    winSLAM2d.plotEllipse( m_xkk[0], m_xkk[1], circle, 3, "b-2", "robot circle" );
    vector<float> plotX, plotY;
    plotX.push_back(m_xkk[0]);
    plotX.push_back(m_xkk[0] + 0.3*cos(m_xkk[2]));
    plotY.push_back((m_xkk[1]));
    plotY.push_back(m_xkk[1] + 0.3*sin(m_xkk[2]));
    winSLAM2d.plot( plotX, plotY,"g-4","angle");
    winSLAM2d.hold_on();
    winSLAM2d.plotEllipse(m_xkk[0], m_xkk[1], COVXY, 1, "c:1", "a" );
    winSLAM2d.hold_on();
    winSLAM2d.plot( pathx, pathy,"m.4","path");

}

void mrptKF2d::printInfo(){
    vector<TPoint2D>	 LMs;
    map<unsigned int, CLandmark::TLandmarkID>    LM_IDs;
    CMatrixDouble  fullCov;
    CVectorDouble  fullState;
    CPosePDFGaussian robotPose;
    KFMatrix_OxO out_R;
    this->OnGetObservationNoise(out_R);

    getCurrentState( robotPose,LMs,LM_IDs,fullState,fullCov );
    cout << "Mean pose: " << endl << robotPose.mean << endl;
    cout << "# of landmarks in the map: " << LMs.size() << endl;
    cout << "STATES: ======================== \n" << m_xkk << endl;
    cout << "COV ============================ \n" << fullCov << endl;
    cout << "Current path size " <<  pathx.size() << ", " <<  pathy.size() << endl;
    cout << "OUT_R =========================== \n " << out_R << endl;
}

void mrptKF2d::getPose(double &x, double &y, double &theta){
    x = m_xkk[0];
    y = m_xkk[1];
    theta = m_xkk[2];
}
