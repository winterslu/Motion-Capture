#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
    LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
    BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
    for(int i=0; i<3; i++)
        angles[i] *= M_PI / 180;
    R[0] = cos(angles[1]) * cos(angles[2]);
    R[1] = sin(angles[0]) * sin(angles[1]) * cos(angles[2]) - cos(angles[0]) * sin(angles[2]);
    R[2] = cos(angles[0]) * sin(angles[1]) * cos(angles[2]) + sin(angles[0]) * sin(angles[2]);
    R[3] = cos(angles[1]) * sin(angles[2]);
    R[4] = sin(angles[0]) * sin(angles[1]) * sin(angles[2]) + cos(angles[0]) * cos(angles[2]);
    R[5] = cos(angles[0]) * sin(angles[1]) * sin(angles[2]) - sin(angles[0]) * cos(angles[2]);
    R[6] = -sin(angles[1]);
    R[7] = sin(angles[0]) * cos(angles[1]);
    R[8] = cos(angles[0]) * cos(angles[1]);
}

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1
    
    int startKeyframe = 0;
    
    //TimeCounter
    timeCounter.StartCounter();
    
    while (startKeyframe + N + 1 < inputLength)
    {
        
        int endKeyframe = startKeyframe + N + 1;
        
        Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture * endPosture = pInputMotion->GetPosture(endKeyframe);
        
        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);
        
        // interpolate in between
        for(int frame=1; frame<=N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N+1);
            
            // interpolate root position
            interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;
            
            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
                interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;
            
            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }
        
        startKeyframe = endKeyframe;
    }
    timeCounter.StopCounter();
    printf("TimeCount: %fs\n",timeCounter.GetElapsedTime());
    
    
    for(int frame=startKeyframe+1; frame<inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    // students should implement this
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1
    
    int startKeyframe = 0;
    
    //TimeCounter
    timeCounter.StartCounter();
    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;
        
        Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture * endPosture = pInputMotion->GetPosture(endKeyframe);
        
        // For Beizer
        int last = startKeyframe - N - 1;
        if(startKeyframe == 0)
            last = startKeyframe;
        int next = endKeyframe + N + 1;
        if(endKeyframe + N + 1>= inputLength)
            next = endKeyframe;
        vector * lastRotation = pInputMotion->GetPosture(last)->bone_rotation;
        vector * nextRotation = pInputMotion->GetPosture(next)->bone_rotation;
        
        
        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);
        
        // interpolate in between
        for(int frame=1; frame<=N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N+1);
            
            // interpolate root position
            interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;
            
            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                Quaternion<double> qStart, qEnd, qInterp, qLast, qNext;
                vector vStart, vEnd, vLast, vNext;
                
                vStart = startPosture->bone_rotation[bone];
                vEnd = endPosture->bone_rotation[bone];
                // For Beizer
                vLast = lastRotation[bone];
                vNext = nextRotation[bone];
                // an
                vLast = vStart + vStart - vLast;
                vLast = (vLast + vEnd) * 0.5;
                // bn+1
                vector vTemp = vEnd + vEnd - vStart;
                vNext = (vTemp + vNext) * 0.5;
                vNext = vEnd + vEnd - vNext;
                // 1/3
                vLast = vStart + (vLast - vStart) / 3;
                vNext = vEnd + (vNext - vEnd) / 3;
                
                interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t, vStart, vLast, vNext, vEnd);
            }
            
            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }
        
        startKeyframe = endKeyframe;
    }
    timeCounter.StopCounter();
    printf("TimeCount: %fs\n",timeCounter.GetElapsedTime());
    
    for(int frame=startKeyframe+1; frame<inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    // students should implement this
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1
    
    int startKeyframe = 0;
    //TimeCounter
    timeCounter.StartCounter();
    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;
        
        Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture * endPosture = pInputMotion->GetPosture(endKeyframe);
        
        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);
        
        // interpolate in between
        for(int frame=1; frame<=N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N+1);
            
            // interpolate root position
            interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;
            
            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                double start_angles[3], end_angles[3], interp_angles[3];
                Quaternion<double> qStart, qEnd, qInterp;
                
                startPosture->bone_rotation[bone].getValue(start_angles);
                endPosture->bone_rotation[bone].getValue(end_angles);
                
                
                Euler2Quaternion(start_angles, qStart);
                Euler2Quaternion(end_angles, qEnd);
                qInterp = Slerp(t, qStart, qEnd);
                
                Quaternion2Euler(qInterp, interp_angles);
                
                interpolatedPosture.bone_rotation[bone].setValue(interp_angles);
            }
            
            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }
        
        startKeyframe = endKeyframe;
    }
    
    timeCounter.StopCounter();
    printf("TimeCount: %fs\n",timeCounter.GetElapsedTime());
    
    for(int frame=startKeyframe+1; frame<inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
    
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    // students should implement this
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1
    
    int startKeyframe = 0;
    //TimeCounter
    timeCounter.StartCounter();
    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;
        
        Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture * endPosture = pInputMotion->GetPosture(endKeyframe);
        
        // For Beizer
        int last = startKeyframe - N - 1;
        if(startKeyframe == 0)
            last = startKeyframe;
        int next = endKeyframe + N + 1;
        if(endKeyframe + N + 1>= inputLength)
            next = endKeyframe;
        vector * lastRotation = pInputMotion->GetPosture(last)->bone_rotation;
        vector * nextRotation = pInputMotion->GetPosture(next)->bone_rotation;
        
        
        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);
        
        // interpolate in between
        for(int frame=1; frame<=N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N+1);
            
            // interpolate root position
            interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;
            
            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                double start_angles[3], end_angles[3], interp_angles[3], last_angles[3], next_angles[3];
                Quaternion<double> qStart, qEnd, qInterp, qLast, qNext;
                
                startPosture->bone_rotation[bone].getValue(start_angles);
                endPosture->bone_rotation[bone].getValue(end_angles);
                // For Beizer
                lastRotation[bone].getValue(last_angles);
                nextRotation[bone].getValue(next_angles);
                
                Euler2Quaternion(start_angles, qStart);
                Euler2Quaternion(end_angles, qEnd);
                Euler2Quaternion(last_angles, qLast);
                Euler2Quaternion(next_angles, qNext);
                
                // an
                qLast = Double(qLast, qStart);
                qLast = Slerp(0.5, qLast, qEnd);
                // bn+1
                Quaternion<double> qTemp = Double(qStart, qEnd);
                qNext = Slerp(0.5, qTemp, qNext);
                qNext = Double(qNext, qEnd);
                // 1/3
                qLast = Slerp(1.0/3, qStart, qLast);
                qNext = Slerp(1.0/3, qEnd, qNext);
                
                qInterp =  DeCasteljauQuaternion(t, qStart, qLast, qNext, qEnd);
                
                Quaternion2Euler(qInterp, interp_angles);
                
                interpolatedPosture.bone_rotation[bone].setValue(interp_angles);
            }
            
            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }
        
        startKeyframe = endKeyframe;
    }
    timeCounter.StopCounter();
    printf("TimeCount: %fs\n",timeCounter.GetElapsedTime());
    
    
    for(int frame=startKeyframe+1; frame<inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q) 
{
    // students should implement this
    double r[9];
    Euler2Rotation(angles, r);
    
    //q = q.Matrix2Quaternion(r);
    double trace = r[0] + r[4] + r[8];
    if( trace > 0 )
    {
        double s = 0.5f / sqrt(trace+ 1.0f);
        q.Set(0.25f / s, (r[7]-r[5]) * s, (r[2]-r[6]) * s, (r[3]-r[1]) * s);
    } else {
        if ( r[0] > r[4] && r[0] > r[8] ) {
            double s = 2.0f * sqrt( 1.0f + r[0] - r[4] - r[8]);
            q.Set((r[7]-r[5]) / s, 0.25f * s, (r[1]+r[3]) / s, (r[2]+r[6]) / s);
        } else if (r[4] > r[8]) {
            double s = 2.0f * sqrtf( 1.0f + r[4] - r[0] - r[8]);
            q.Set((r[2]-r[6]) / s, (r[1]+r[3]) / s, 0.25f * s, (r[5]+r[7]) / s);
        } else {
            double s = 2.0f * sqrtf( 1.0f + r[8] - r[0] - r[4] );
            q.Set((r[3]-r[1]) / s, (r[2]+r[6]) / s, (r[5]+r[7]) / s, 0.25f * s);
        }
    }

//    ******************************************
//    Alternative Method on Euler to Quaternion
//    ******************************************
//    double roll = angles[0] *= M_PI / 180;
//    double pitch = angles[1] *= M_PI / 180;
//    double yaw = angles[2] *= M_PI / 180;
//    
//    double cyaw, cpitch, croll, syaw, spitch, sroll;
//    double cyawcpitch, syawspitch, cyawspitch, syawcpitch;
//    
//    cyaw = cos(0.5f * yaw);
//    cpitch = cos(0.5f * pitch);
//    croll = cos(0.5f * roll);
//    syaw = sin(0.5f * yaw);
//    spitch = sin(0.5f * pitch);
//    sroll = sin(0.5f * roll);
//    
//    cyawcpitch = cyaw * cpitch;
//    syawspitch = syaw * spitch;
//    cyawspitch = cyaw * spitch;
//    syawcpitch = syaw * cpitch;
//    
//    q.Set((double)(cyawcpitch * croll + syawspitch * sroll), 
//          (double)(cyawcpitch * sroll - syawspitch * croll), 
//          (double)(cyawspitch * croll + syawcpitch * sroll), 
//          (double)(syawcpitch * croll - cyawspitch * sroll));
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3]) 
{
    // students should implement this
    double r[3][3];
    double q00, q11, q22, q33;
    double tmp;
    vector u;
    q00 = q.Gets() * q.Gets();
    q11 = q.Getx() * q.Getx();
    q22 = q.Gety() * q.Gety();
    q33 = q.Getz() * q.Getz();
    
    r[0][0] = q00 + q11 - q22 - q33;
    r[1][0] = 2 * (q.Getx()*q.Gety() + q.Gets()*q.Getz());
    r[2][0] = 2 * (q.Getx()*q.Getz() - q.Gets()*q.Gety());
    r[2][1] = 2 * (q.Gety()*q.Getz() + q.Gets()*q.Getx());
    r[2][2] = q00 - q11 - q22 + q33;
    
    tmp = fabs(r[2][0]);
    if(tmp > 0.999999)
    {
        r[0][1] = 2 * (q.Getx()*q.Gety() - q.Gets()*q.Getz());
        r[0][2] = 2 * (q.Getx()*q.Getz() + q.Gets()*q.Gety());
        angles[0] = 0.0f * 180 / M_PI;
        angles[1] = (-(M_PI/2)*r[2][0]/tmp) * 180 / M_PI;
        angles[2] = (atan2(-r[0][1], -r[2][0]*r[0][2])) * 180 / M_PI;
        return;
    }
    else
    {
        angles[0] = (atan2(r[2][1], r[2][2])) * 180 / M_PI;
        angles[1] = (asin(-r[2][0])) * 180 / M_PI;
        angles[2] = (atan2(r[1][0], r[0][0])) * 180 / M_PI;
        return;
    }
    
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
    // students should implement this
    Quaternion<double> result;
    Quaternion<double> q3;
    //float dot = quaternion::dot(qStart, qEnd_);
    float dot = qStart.Gets()*qEnd_.Gets() + 
                qStart.Getx()*qEnd_.Getx() + 
                qStart.Gety()*qEnd_.Gety() + 
                qStart.Getz()*qEnd_.Getz();
    
    /*	dot = cos(theta)
     if (dot < 0), q1 and q2 are more than 90 degrees apart,
     so we can invert one to reduce spinning	*/
    if (dot < 0)
    {
        dot = -dot;
        q3.Set(-qEnd_.Gets(), -qEnd_.Getx(), -qEnd_.Gety(), -qEnd_.Getz());
        //q3 = -qEnd_;
    } else q3 = qEnd_;
    
    float angle = acosf(dot);
    if(angle == 0)
        return qEnd_;
    
//    float a = sinf((1-t)*angle)/sinf(angle);
//    float b = sinf(t*angle)/sinf(angle);
//    Quaternion<double> aa = qStart * a;
//    Quaternion<double> bb = q3 * b;
//    result = aa + bb;
    result = (sinf((1-t)*angle)/sinf(angle)) * qStart + (sinf(t*angle)/sinf(angle)) * q3;
    result.Normalize();
    return result;
    
    /* Optional Method to do Slerp
    if (dot < 0.95f)
    {
        float angle = acosf(dot);
        result = (qStart * sinf((1-t)*angle) + q3 * sinf(t*angle)) / sinf(angle);
        return result;
    } else // if the angle is small, use linear interpolation
    {
        result = qStart*(1-t) + q3*t;
        result.Normalize();
        return result;
    }*/
}




Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
    // students should implement this
    Quaternion<double> result;
    result = 2 * (p.Gets() * q.Gets() + p.Getx() * q.Getx() + p.Gety() * q.Gety() + p.Getz() * q.Getz()) * q - p;
    
    return result;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
    // students should implement this
    vector result;
    vector V0, V1, V2, R0, R1;
    
    V0 = p0 * (1-t) + p1 * t;
    V1 = p1 * (1-t) + p2 * t;
    V2 = p2 * (1-t) + p3 * t;
    R0 = V0 * (1-t) + V1 * t;
    R1 = V1 * (1-t) + V2 * t;
    result = R0 * (1-t) + R1 * t;
    
    return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
    // students should implement this
    Quaternion<double> result;
    Quaternion<double> Q0, Q1, Q2, R0, R1;
    
    Q0 = Slerp(t, p0, p1);
    Q1 = Slerp(t, p1, p2);
    Q2 = Slerp(t, p2, p3);
    R0 = Slerp(t, Q0, Q1);
    R1 = Slerp(t, Q1, Q2);
    result = Slerp(t, R0, R1);
    
    return result;
}

