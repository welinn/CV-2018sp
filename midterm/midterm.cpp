#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

Mat src1, src2;
vector<Point2f> pointL, pointR;
vector<Point3f> normalL, normalR;
Mat T, TP, F, epipoleL, epipoleR, Te_L, Te_R;
Mat R_L, R_R, RTe_L, RTe_R, H_L, H_R;
float error_F;

void setPoints();
void calT();
void normalizePoint();
void calF(int = 0);
bool calError();
void calEpipole();
void calR();
void calHa();

int main(int argc, char* argv[]){

  Mat dst1, dst2;
  int width, height;

  src1 = imread("./L.JPG");
  if(!src1.data){
    printf("open image L fail\n");
    return -1;
  }
  src2 = imread("./R.JPG");
  if(!src2.data){
    printf("open image R fail\n");
    return -1;
  }

  error_F = 1e-3;
  setPoints();
  calT();
  normalizePoint();
  calF();
  while(!calError()){
    calF();
  }
  calF(1); //ch6-1, P.57, t = 0
  calEpipole();

  //cal T
  Mat trans = (Mat_<float>(3, 3) <<
      1, 0, -src1.cols,
      0, 1, -src1.rows,
      0, 0, 1);

  Te_L = trans * epipoleL;
  Te_R = trans * epipoleR;

  calR();

  //cal G
  Mat GL = (Mat_<float>(3, 3) <<
      1, 0, 0,
      0, 1, 0,
      -1 / RTe_L.at<float>(0, 0), 0, 1);

  Mat GR = (Mat_<float>(3, 3) <<
      1, 0, 0,
      0, 1, 0,
      -1 / RTe_R.at<float>(0, 0), 0, 1);
  //calH
  H_L = GL * R_L * trans;
  H_R = GR * R_R * trans;

  //TODO
  calHa();
  //imwrite("output.jpg", dst);
  //imshow("test", dst);
  //waitKey();

  return 0;
}

void setPoints(){
  pointL.push_back(Point2f(1634, 1329)); pointL.push_back(Point2f(1662, 1340));
  pointL.push_back(Point2f(1848, 1627)); pointL.push_back(Point2f(3806,  634));
  pointL.push_back(Point2f(2090, 1603)); pointL.push_back(Point2f(2106, 1902));
  pointL.push_back(Point2f(2239, 1940)); pointL.push_back(Point2f(2633, 1546));
  pointL.push_back(Point2f(2469, 1494)); pointL.push_back(Point2f(2427, 1507));

  pointR.push_back(Point2f(1942, 1138)); pointR.push_back(Point2f(2027, 1155));
  pointR.push_back(Point2f(2092, 1578)); pointR.push_back(Point2f(2836,  677));
  pointR.push_back(Point2f(2464, 1560)); pointR.push_back(Point2f(2364, 1963));
  pointR.push_back(Point2f(2606, 2016)); pointR.push_back(Point2f(2980, 1525));
  pointR.push_back(Point2f(2665, 1453)); pointR.push_back(Point2f(2668, 1464));

}

void calT(){
  float xAvg, yAvg, length;
  Mat tmp1, tmp2;

  //L img
  for(int i = 0, xAvg = 0, yAvg = 0, length = 0; i < pointL.size(); i++){
    xAvg += pointL[i].x;
    yAvg += pointL[i].y;
    length += sqrt(pointL[i].x * pointL[i].x + pointL[i].y * pointL[i].y);
  }
  xAvg /= pointL.size();
  yAvg /= pointL.size();
  length /= pointL.size();

  tmp1 = (Mat_<float>(3, 3) <<
      sqrt(2) / length, 0, 0,
      0, sqrt(2) / length, 0,
      0,                0, 1);
  tmp2 = (Mat_<float>(3, 3) <<
      1, 0, -xAvg,
      0, 1, -yAvg,
      0, 0,     1);

  T = tmp1 * tmp2;

  //R img
  for(int i = 0, xAvg = 0, yAvg = 0, length = 0; i < pointR.size(); i++){
    xAvg += pointR[i].x;
    yAvg += pointR[i].y;
    length += sqrt(pointR[i].x * pointR[i].x + pointR[i].y * pointR[i].y);
  }
  xAvg /= pointR.size();
  yAvg /= pointR.size();
  length /= pointR.size();

  tmp1 = (Mat_<float>(3, 3) <<
      sqrt(2) / length, 0, 0,
      0, sqrt(2) / length, 0,
      0,                0, 1);
  tmp2 = (Mat_<float>(3, 3) <<
      1, 0, -xAvg,
      0, 1, -yAvg,
      0, 0,     1);

  TP = tmp1 * tmp2;

}

void normalizePoint(){
  Mat norL, norR;
  for(int i = 0; i < pointL.size(); i++){
    norL = (Mat_<float>(3, 1) << pointL[i].x, pointL[i].y, 1);
    norR = (Mat_<float>(3, 1) << pointR[i].x, pointR[i].y, 1);
    norL = T  * norL;
    norR = TP * norR;
    normalL.push_back(Point3f(norL.at<float>(0, 0), norL.at<float>(1, 0), norL.at<float>(2, 0)));
    normalR.push_back(Point3f(norR.at<float>(0, 0), norR.at<float>(1, 0), norR.at<float>(2, 0)));
  }
}

void calF(int mode){
  int size = normalL.size();
  Mat A(size, 9, CV_32F);
  for(int i = 0; i < size; i++){
    A.at<float>(i, 0) = normalR[i].x * normalL[i].x;
    A.at<float>(i, 1) = normalR[i].x * normalL[i].y;
    A.at<float>(i, 2) = normalR[i].x;
    A.at<float>(i, 3) = normalL[i].x * normalR[i].y;
    A.at<float>(i, 4) = normalL[i].y * normalR[i].y;
    A.at<float>(i, 5) = normalR[i].y;
    A.at<float>(i, 6) = normalL[i].x;
    A.at<float>(i, 7) = normalL[i].y;
    A.at<float>(i, 8) = 1;
  }
  Mat w, u, vt;
  SVD::compute(A, w, u, vt, SVD::FULL_UV);  //SVD 分解
  Mat Fh = (Mat_<float>(3, 3) <<
      vt.at<float>(8, 0), vt.at<float>(8, 1), vt.at<float>(8, 2),
      vt.at<float>(8, 3), vt.at<float>(8, 4), vt.at<float>(8, 5),
      vt.at<float>(8, 6), vt.at<float>(8, 7), vt.at<float>(8, 8));

  if(mode == 0)
    F = TP.t() * Fh * T;
  else{
    SVD::compute(Fh, w, u, vt, SVD::FULL_UV);
    w.at<float>(2, 2) = 0;
    F = TP.t() * (u * w * vt) * T;
  }
}

bool calError(){
  Mat pL, pR;
  float err, max = -10000;
  int index;

  for(int i = 0; i < normalL.size(); i++){
    pL = (Mat_<float>(3, 1) << normalL[i].x, normalL[i].y, normalL[i].z);
    pR = (Mat_<float>(1, 3) << normalR[i].x, normalR[i].y, normalR[i].z);
    Mat errMat = pR * F * pL;
    err = abs(errMat.at<float>(0, 0));
    if(max < err){
      max = err;
      index = i;
    }
  }
  if(max > error_F){
    normalL[index] = normalL[normalL.size()];
    normalR[index] = normalR[normalR.size()];
    //pointL[index] = pointL[pointL.size()];
    //pointR[index] = pointR[pointR.size()];
    normalL.pop_back();
    normalR.pop_back();
    //pointL.pop_back();
    //pointR.pop_back();
    return false;
  }
  return true;
}

void calEpipole(){
  Mat l1 = (Mat_<float>(3, 1) << normalL[0].x, normalL[0].y, normalL[0].z);
  Mat l2 = (Mat_<float>(3, 1) << normalL[1].x, normalL[1].y, normalL[1].z);
  Mat r1 = (Mat_<float>(3, 1) << normalR[0].x, normalR[0].y, normalR[0].z);
  Mat r2 = (Mat_<float>(3, 1) << normalR[1].x, normalR[1].y, normalR[1].z);
/*
  Mat l1 = (Mat_<float>(3, 1) << pointL[0].x, pointL[0].y, pointL[0].z);
  Mat l2 = (Mat_<float>(3, 1) << pointL[1].x, pointL[1].y, pointL[1].z);
  Mat r1 = (Mat_<float>(3, 1) << pointR[0].x, pointR[0].y, pointR[0].z);
  Mat r2 = (Mat_<float>(3, 1) << pointR[1].x, pointR[1].y, pointR[1].z);
*/
  epipoleL = (F.t() * r1).cross(F.t() * r2);
  epipoleR = (F     * l1).cross(F     * l2);
  epipoleL /= epipoleL.at<float>(2, 0);
  epipoleR /= epipoleR.at<float>(2, 0);
}

void calR(){
  float angleL = -1 * atan(Te_L.at<float>(1, 0) / Te_L.at<float>(0, 0));
  float angleR = -1 * atan(Te_R.at<float>(1, 0) / Te_R.at<float>(0, 0));

  R_L = (Mat_<float>(3, 3) <<
      cos(angleL), -sin(angleL), 0,
      sin(angleL),  cos(angleL), 0,
                0,            0, 1);
  RTe_L = R_L * Te_L;

  R_R = (Mat_<float>(3, 3) <<
      cos(angleR), -sin(angleR), 0,
      sin(angleR),  cos(angleR), 0,
                0,            0, 1);
  RTe_R = R_R * Te_R;
}
