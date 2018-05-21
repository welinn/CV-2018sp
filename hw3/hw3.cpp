#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

void setSquare(Mat*, Mat*, Mat*, Mat*);
void setA(Mat*, Mat, Mat, Mat);
void getInvOmega(Mat*, Mat);
void calK(Mat*, Mat);

int main(int argc, char *argv[]){

  Mat A_square, B_square, C_square, unit_square;

  setSquare(&A_square, &B_square, &C_square, &unit_square);
  //cal homography
  Mat ha = findHomography(unit_square, A_square);
  Mat hb = findHomography(unit_square, B_square);
  Mat hc = findHomography(unit_square, C_square);
  //transpose
  Mat hap = ha.t();
  Mat hbp = hb.t();
  Mat hcp = hc.t();

  Mat A;
  setA(&A, hap, hbp, hcp);

  Mat inw;
  getInvOmega(&inw, A);
  inw /= inw.at<double>(2, 2);//ch7-p.45

  Mat k;
  calK(&k, inw);

  Mat ra = k.inv() * ha;
  Mat rb = k.inv() * hb;
  Mat rc = k.inv() * hc;



  return 0;
}

void setSquare(Mat *a, Mat *b, Mat *c, Mat *u){

  *a = (Mat_<double>(4, 2) << );
  *b = (Mat_<double>(4, 2) << );
  *c = (Mat_<double>(4, 2) << );
  //左上,左下,右下,右上
  *u = (Mat_<double>(4, 2) << 0, 0,  0, 50,  50, 50,  50, 0);

}

void setA(Mat *A, Mat a, Mat b, Mat c){

  double a00, a01, a02, a10, a11, a12;
  double b00, b01, b02, b10, b11, b12;
  double c00, c01, c02, c10, c11, c12;
  a00 = a.at<double>(0, 0); a01 = a.at<double>(0, 1); a02 = a.at<double>(0, 2);
  a10 = a.at<double>(1, 0); a11 = a.at<double>(1, 1); a12 = a.at<double>(1, 2);
  b00 = b.at<double>(0, 0); b01 = b.at<double>(0, 1); b02 = b.at<double>(0, 2);
  b10 = b.at<double>(1, 0); b11 = b.at<double>(1, 1); b12 = b.at<double>(1, 2);
  c00 = c.at<double>(0, 0); c01 = c.at<double>(0, 1); c02 = c.at<double>(0, 2);
  c10 = c.at<double>(1, 0); c11 = c.at<double>(1, 1); c12 = c.at<double>(1, 2);


  *A = (Mat_<double>(6, 6) <<
      a00*a10, a00*a11 + a01*a10, a00*a12 + a02*a10, a01*a11, a01*a12 + a02*a11, a02*a12,
      a00*a00 - a10*a10, 2*(a00*a01 - a10*a11), 2*(a00*a02 - a10*a12), a01*a01 - a11*a11, 2*(a01*a02 - a11*a12), a02*a02 - a12*a12,
      b00*b10, b00*b11 + b01*b10, b00*b12 + b02*b10, b01*b11, b01*b12 + b02*b11, b02*b12,
      b00*b00 - b10*b10, 2*(b00*b01 - b10*b11), 2*(b00*b02 - b10*b12), b01*b01 - b11*b11, 2*(b01*b02 - b11*b12), b02*b02 - b12*b12,
      c00*c10, c00*c11 + c01*c10, c00*c12 + c02*c10, c01*c11, c01*c12 + c02*c11, c02*c12,
      c00*c00 - c10*c10, 2*(c00*c01 - c10*c11), 2*(c00*c02 - c10*c12), c01*c01 - c11*c11, 2*(c01*c02 - c11*c12), c02*c02 - c12*c12);
}

void getInvOmega(Mat* inw, Mat A){
  //SVD
  Mat u, s, vt;
  SVD::compute(A, u, s, vt, SVD::FULL_UV);
  vt = vt.t();
  //omega
  Mat w = (Mat_<double>(3, 3) <<
      vt.at<double>(0, 5), vt.at<double>(1, 5), vt.at<double>(2, 5),
      vt.at<double>(1, 5), vt.at<double>(3, 5), vt.at<double>(4, 5),
      vt.at<double>(2, 5), vt.at<double>(4, 5), vt.at<double>(5, 5));
  *inw = w.inv();
}

void calK(Mat* k, Mat inw){

  double a, b, c, d, e;
  c = inw.at<double>(0, 2);
  e = inw.at<double>(1, 2);
  d = sqrt(inw.at<double>(1, 1) - e*e);
  b = (inw.at<double>(0,1) - c*e) / d;
  a = sqrt(inw.at<double>(0, 0) - b*b - c*c);
  *k = (Mat_<double>(3, 3) <<
      a, b, c,
      0, d, e,
      0, 0, 1);
}
