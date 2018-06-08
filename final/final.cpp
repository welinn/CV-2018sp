#include <fstream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

void setPara(Mat*, Mat*, Mat*, Mat*, Mat*);
void setP(Mat, Mat, Mat*, Mat*, Mat*, Mat*, Mat*, Mat*);

int main(int argc, char *argv[]){

  fstream dst;
  dst.open("output.xyz", ios::out);
  int num = 293; //picture 數量
  char fileName[20];

  Mat leftK, leftRt, rightK, rightRt, F;
  setPara(&leftK, &leftRt, &rightK, &rightRt, &F);

  Mat leftP = leftK * leftRt;    //左圖P
  Mat rightP = rightK * rightRt; //右圖P

  Mat p1, p2, p3, pp1, pp2, pp3;
  setP(leftP, rightP, &p1, &p2, &p3, &pp1, &pp2, &pp3);

  for(int i = 0; i < num; i++){
    sprintf(fileName, "./L/L%03d.JPG", i);
    Mat srcL = imread(fileName, 0);
    sprintf(fileName, "./R/R%03d.JPG", i);
    Mat srcR = imread(fileName, 0);

    //check all pixel
    for(int x = 0; x < srcL.rows; x++){
      int max = -1;
      int index = -1;
      for(int y = 0; y < srcL.cols; y++){
        if(srcL.at<uchar>(x, y) > 128 && srcL.at<uchar>(x, y) > max){
          max = srcL.at<uchar>(x, y);
          index = y;
        }
      }
      if(index != -1){ //左圖有點
        Mat leftPt = (Mat_<double>(3, 1) << index, x, 1);
        Mat lineR = F * leftPt; //epipolar line
        double tmp1 = -lineR.at<double>(2, 0) / lineR.at<double>(1, 0);
        double tmp2 = -(lineR.at<double>(0, 0) * srcR.cols + lineR.at<double>(2, 0)) / lineR.at<double>(1, 0);

        int hMax = tmp1 > tmp2 ? (tmp1 > srcR.rows ? srcR.rows : tmp1) : (tmp2 > srcR.rows ? srcR.rows : tmp2);
        int hMin = tmp1 < tmp2 ? (tmp1 < 0 ? 0 : tmp1) : (tmp2 < 0 ? 0 : tmp2);

        int maxR = -1;
        int indexRh = -1, indexRw = -1;
        for(int h = hMin; h < hMax; h++){
          for(int w = 0; w < srcR.cols; w++){
            if(srcR.at<uchar>(h, w) > 128 && srcR.at<uchar>(h, w) > maxR){
              //check point is on epipolar line
              if(abs(lineR.at<double>(0, 0) * w + lineR.at<double>(1, 0) * h + lineR.at<double>(2, 0)) < 1){
                maxR = srcR.at<uchar>(h, w);
                indexRh = h;
                indexRw = w;
              }
            }
          }
        }
        if(indexRh != -1){ //右圖有點
          int u = leftPt.at<double>(0, 0);
          int v = leftPt.at<double>(1, 0);
          int u2 = indexRw;
          int v2 = indexRh;

          //ch8 p.24
          Mat a1 = u * p3.t() - p1.t();
          Mat a2 = v * p3.t() - p2.t();
          Mat a3 = u2 * pp3.t() - pp1.t();
          Mat a4 = v2 * pp3.t() - pp2.t();
          Mat A, Atmp1, Atmp2;
          hconcat(a1, a2, Atmp1);
          hconcat(a3, a4, Atmp2);
          hconcat(Atmp1, Atmp2, A);

          Mat U, S, vt;
          SVD::compute(A, U, S, vt, SVD::FULL_UV);

          Mat V = vt.t();
          Mat X = V.colRange(4, 5);
          X /= X.at<double>(3, 0); //normalize

          Mat err = A * X;
          if(err.at<double>(0, 0)*err.at<double>(0, 0) + err.at<double>(1, 0)*err.at<double>(1, 0) + err.at<double>(2, 0)*err.at<double>(2, 0) + err.at<double>(3, 0)*err.at<double>(3, 0) < 50){
            dst << X.at<double>(0, 0) << " " << X.at<double>(1, 0) << " " << X.at<double>(2, 0) << endl;
          }

        }


      }
    }
  }

  dst.close();
  return 0;
}

void setPara(Mat *leftK, Mat *leftRt, Mat *rightK, Mat *rightRt, Mat *F){
  *leftK = (Mat_<double>(3, 3) <<
      1035.278669095568, 0.000000000000, 295.500377771516,
      0.000000000000, 1034.880664685675, 598.224722223280,
      0.0, 0.0, 1.0);
  *leftRt = (Mat_<double>(3, 3) <<
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0);
  *rightK = (Mat_<double>(3, 3) <<
      1036.770200759934, 0.000000000000, 403.040387412710,
      0.000000000000, 1037.186415753241, 612.775486819306,
      0.0, 0.0, 1.0);
  *rightRt = (Mat_<double>(3, 3) <<
       0.958173249509,  0.009400631103,  0.286034354684, -69.855978076557,
      -0.009648701145,  0.999953303246, -0.000542119475,  0.110435878514,
      -0.286026094074, -0.002240415626,  0.958219209809,  14.517584144224);
  *F = (Mat_<double>(3, 3) <<
      -0.000000076386,  0.000003333642, -0.001745446202,
       0.000000105635, -0.000000103805, -0.011352133262,
      -0.000285872783,  0.010010406460, 1.000000000000);


}

void setP(Mat leftP, Mat rightP, Mat *p1, Mat *p2, Mat *p3, Mat *pp1, Mat *pp2, Mat *pp3){

  *p1 = (Mat_<double>(4, 1) <<
      leftP.at<double>(0, 0), leftP.at<double>(0, 1), leftP.at<double>(0, 2), leftP.at<double>(0, 3));
  *p2 = (Mat_<double>(4, 1) <<
      leftP.at<double>(1, 0), leftP.at<double>(1, 1), leftP.at<double>(1, 2), leftP.at<double>(1, 3));
  *p3 = (Mat_<double>(4, 1) <<
      leftP.at<double>(2, 0), leftP.at<double>(2, 1), leftP.at<double>(2, 2), leftP.at<double>(2, 3));

  *pp1 = (Mat_<double>(4, 1) <<
      rightP.at<double>(0, 0), rightP.at<double>(0, 1), rightP.at<double>(0, 2), rightP.at<double>(0, 3));
  *pp2 = (Mat_<double>(4, 1) <<
      rightP.at<double>(1, 0), rightP.at<double>(1, 1), rightP.at<double>(1, 2), rightP.at<double>(1, 3));
  *pp3 = (Mat_<double>(4, 1) <<
      rightP.at<double>(2, 0), rightP.at<double>(2, 1), rightP.at<double>(2, 2), rightP.at<double>(2, 3));
}
