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

  Mat color = imread("L/L_Color.JPG", 1);
  if(!color.data){
    printf("open img fail\n");
    return -1;
  }

  Mat leftK, leftRt, rightK, rightRt, F;
  setPara(&leftK, &leftRt, &rightK, &rightRt, &F);

  Mat leftP = leftK * leftRt;    //左圖P
  Mat rightP = rightK * rightRt; //右圖P

  Mat p1, p2, p3, pp1, pp2, pp3;
  setP(leftP, rightP, &p1, &p2, &p3, &pp1, &pp2, &pp3);

  for(int i = 0; i < num; i++){
    sprintf(fileName, "./L/L%03d.JPG", i);
    Mat srcL = imread(fileName, 0);
    if(!srcL.data){
      printf("open img fail\n");
      return -1;
    }
    sprintf(fileName, "./R/R%03d.JPG", i);
    Mat srcR = imread(fileName, 0);
    if(!srcR.data){
      printf("open img fail\n");
      return -1;
    }

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
        Mat lineR = F * leftPt; //epipolar line ( ax + by + cz = 0)
        double y1 = -lineR.at<double>(2, 0) / lineR.at<double>(1, 0);  //x = 0
        double y2 = -(lineR.at<double>(0, 0) * srcR.cols + lineR.at<double>(2, 0)) / lineR.at<double>(1, 0);  //x = srcR.cols

        //可能的高度範圍
        int hMax = y1 > y2 ? (y1 > srcR.rows ? srcR.rows : y1) : (y2 > srcR.rows ? srcR.rows : y2);
        int hMin = y1 < y2 ? (y1 < 0 ? 0 : y1) : (y2 < 0 ? 0 : y2);

        double minLen = 100;
        int indexRh = -1, indexRw = -1;
        for(int h = hMin; h < hMax; h++){
          for(int w = 0; w < srcR.cols; w++){
            if(srcR.at<uchar>(h, w) > 128){
              //check point is on epipolar line
              Mat rightPt = (Mat_<double>(3, 1) << w, h, 1);
              Mat len = rightPt.t() * F * leftPt;
              if(abs(len.at<double>(0, 0)) < 0.1 && abs(len.at<double>(0, 0)) < minLen){
                minLen = len.at<double>(0, 0);
                indexRh = h;
                indexRw = w;
              }
            }
          }
        }
        if(indexRh != -1){ //右圖有點

          int u = leftPt.at<double>(0, 0);
          int v = srcL.rows - leftPt.at<double>(1, 0);
          int u2 = indexRw;
          int v2 = srcR.rows - indexRh;

          //ch8 p.24
          Mat a1 = u * p3.t() - p1.t();
          Mat a2 = v * p3.t() - p2.t();
          Mat a3 = u2 * pp3.t() - pp1.t();
          Mat a4 = v2 * pp3.t() - pp2.t();
          Mat A, Atmp1, Atmp2;
          vconcat(a1, a2, Atmp1);
          vconcat(a3, a4, Atmp2);
          vconcat(Atmp1, Atmp2, A);
          Mat U, S, vt;
          SVD::compute(A, U, S, vt, SVD::FULL_UV);

          Mat V = vt.t();
          Mat X = V.colRange(3, 4);
          X /= X.at<double>(3, 0); //normalize

//          Mat err = A * X;
//          if(sqrt(err.at<double>(0, 0)*err.at<double>(0, 0) + err.at<double>(1, 0)*err.at<double>(1, 0) + err.at<double>(2, 0)*err.at<double>(2, 0) + err.at<double>(3, 0)*err.at<double>(3, 0)) < 30000){
            dst << X.at<double>(0, 0) << " " << X.at<double>(1, 0) << " " << -X.at<double>(2, 0) << " " <<
              (double)color.at<Vec3b>(leftPt.at<double>(1, 0), u)[2]/255 << " " <<
              (double)color.at<Vec3b>(leftPt.at<double>(1, 0), u)[1]/255 << " " <<
              (double)color.at<Vec3b>(leftPt.at<double>(1, 0), u)[0]/255 << endl;
              //(int)color.at<Vec3b>(leftPt.at<double>(1, 0), u)[2] << " " <<
              //(int)color.at<Vec3b>(leftPt.at<double>(1, 0), u)[1] << " " <<
              //(int)color.at<Vec3b>(leftPt.at<double>(1, 0), u)[0] << endl;
//          }
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
  *leftRt = (Mat_<double>(3, 4) <<
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0);
  *rightK = (Mat_<double>(3, 3) <<
      1036.770200759934, 0.000000000000, 403.040387412710,
      0.000000000000, 1037.186415753241, 612.775486819306,
      0.0, 0.0, 1.0);
  *rightRt = (Mat_<double>(3, 4) <<
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
