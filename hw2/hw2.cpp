#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

Mat src;
vector<Point> quad1, quad2;
vector<vector<Point> > quad;

void onMouse(int, int, int, int, void*);
Point min_4(Point, Point, Point, Point);
Point max_4(Point, Point, Point, Point);
double crossArea(Point, Point);
Point calVector(Point, Point);
Mat calProjectiveMatrix(int);
double calTotalArea(int);
double calArea(Point, int);

int main(int argc, char* argv[]){

  Mat dst, img;
  int width, height;

  src = imread("./_DSC0001.JPG");
  if(!src.data){
    printf("open image fail\n");
    return -1;
  }
  dst = src.clone();
  img = src.clone();

  width = src.cols;
  height = src.rows;

  namedWindow("Click corner");
  imshow("Click corner", img);
  setMouseCallback("Click corner", onMouse, (void*) &img);
  waitKey();

  //quadrilateral 1
  Mat H = calProjectiveMatrix(0);
  //算要檢查的 pixel 範圍
  Point min1 = min_4(quad1[0], quad1[1], quad1[2], quad1[3]);
  Point max1 = max_4(quad1[0], quad1[1], quad1[2], quad1[3]);
  double area = calTotalArea(0);  //四邊形1的面積
  for(int i = min1.x; i < max1.x; i++){
    for(int j = min1.y; j < max1.y; j++){
      double area2 = calArea(Point(i, j), 0);  //算該點和四角點連線所組成的面積
      if(abs(area - area2) < 5){
        //是要轉換的點
        Mat x = (Mat_<float>(3, 1) << i, j, 1);
        Mat xx = H * x;
        int newx = (int) (xx.at<float>(0, 0) / xx.at<float>(2, 0));
        int newy = (int) (xx.at<float>(1, 0) / xx.at<float>(2, 0));
        dst.at<Vec3b>(i, j) = src.at<Vec3b>(newx, newy);
      }
    }
  }

  //quadrilateral 2
  H = calProjectiveMatrix(1);
  area = calTotalArea(1); //四邊形2的面積
  Point min2 = min_4(quad2[0], quad2[1], quad2[2], quad2[3]);
  Point max2 = max_4(quad2[0], quad2[1], quad2[2], quad2[3]);
  for(int i = min2.x; i < max2.x; i++){
    for(int j = min2.y; j < max2.y; j++){
      double area2 = calArea(Point(i, j), 1);  //算該點和四角點連線所組成的面積
      if(abs(area - area2) < 5){
        //是要轉換的點
        Mat x = (Mat_<float>(3, 1) << i, j, 1);
        Mat xx = H * x;
        int newx = (int) (xx.at<float>(0, 0) / xx.at<float>(2, 0));
        int newy = (int) (xx.at<float>(1, 0) / xx.at<float>(2, 0));
        //dst.at<Vec3b>(newx, newy) = src.at<Vec3b>(i, j);
        dst.at<Vec3b>(i, j) = src.at<Vec3b>(newx, newy);
      }
    }
  }
  imwrite("output.jpg", dst);
  imshow("test", dst);
  waitKey();

  return 0;
}

void onMouse(int event, int y, int x, int flags, void *param){

  static int count = 0;
  Mat img = *((Mat*) param);
  Point input;
  if(event == CV_EVENT_LBUTTONDOWN && count < 4){
    count++;
    input.x = x;
    input.y = y;
    quad1.push_back(input);
    circle(img, Point(y, x), 5, Scalar(0, 255, 0), CV_FILLED);
    imshow("Click corner", img);
  }
  else if(event == CV_EVENT_LBUTTONDOWN){
    count++;
    input.x = x;
    input.y = y;
    quad2.push_back(input);
    circle(img, Point(y, x), 5, Scalar(0, 255, 0), CV_FILLED);
    imshow("Click corner", img);
  }
  if(count == 8){
    destroyWindow("Click corner");
    quad.push_back(quad1);
    quad.push_back(quad2);
  }
}

Point min_4(Point p1, Point p2, Point p3, Point p4){
  Point p;
  p.x = p1.x < p2.x ? (p1.x < p3.x ? (p1.x < p4.x ? p1.x : p4.x) : (p3.x < p4.x ? p3.x : p4.x)) :
                      (p2.x < p3.x ? (p2.x < p4.x ? p2.x : p4.x) : (p3.x < p4.x ? p3.x : p4.x));
  p.y = p1.y < p2.y ? (p1.y < p3.y ? (p1.y < p4.y ? p1.y : p4.y) : (p3.y < p4.y ? p3.y : p4.y)) :
                      (p2.y < p3.y ? (p2.y < p4.y ? p2.y : p4.y) : (p3.y < p4.y ? p3.y : p4.y));
  return p;
}

Point max_4(Point p1, Point p2, Point p3, Point p4){
  Point p;
  p.x = p1.x > p2.x ? (p1.x > p3.x ? (p1.x > p4.x ? p1.x : p4.x) : (p3.x > p4.x ? p3.x : p4.x)) :
                      (p2.x > p3.x ? (p2.x > p4.x ? p2.x : p4.x) : (p3.x > p4.x ? p3.x : p4.x));
  p.y = p1.y > p2.y ? (p1.y > p3.y ? (p1.y > p4.y ? p1.y : p4.y) : (p3.y > p4.y ? p3.y : p4.y)) :
                      (p2.y > p3.y ? (p2.y > p4.y ? p2.y : p4.y) : (p3.y > p4.y ? p3.y : p4.y));
  return p;
}

double crossArea(Point v1, Point v2){
  int x, y, z;
  //let v1 & v2  z = 0
  //x = v1.y * 0 - 0 * v2.y;
  //y = 0 * v2.x - v1.x * 0;
  z = v1.x * v2.y - v1.y * v2.x;
  //return sqrt(x*x + y*y + z*z)/2;
  return abs(z) / 2;
}

Point calVector(Point p1, Point p2){
  return Point(p2.x - p1.x, p2.y - p1.y);
}

Mat calProjectiveMatrix(int no){
  Mat m1, m2;

  m1 = (Mat_<float>(8, 8) <<
      quad[no][0].x, quad[no][0].y, 1,          0,          0, 0, -quad[1-no][0].x * quad[no][0].x, -quad[1-no][0].x * quad[no][0].y,
               0,          0, 0, quad[no][0].x, quad[no][0].y, 1, -quad[1-no][0].y * quad[no][0].x, -quad[1-no][0].y * quad[no][0].y,
      quad[no][1].x, quad[no][1].y, 1,          0,          0, 0, -quad[1-no][1].x * quad[no][1].x, -quad[1-no][1].x * quad[no][1].y,
               0,          0, 0, quad[no][1].x, quad[no][1].y, 1, -quad[1-no][1].y * quad[no][1].x, -quad[1-no][1].y * quad[no][1].y,
      quad[no][2].x, quad[no][2].y, 1,          0,          0, 0, -quad[1-no][2].x * quad[no][2].x, -quad[1-no][2].x * quad[no][2].y,
               0,          0, 0, quad[no][2].x, quad[no][2].y, 1, -quad[1-no][2].y * quad[no][2].x, -quad[1-no][2].y * quad[no][2].y,
      quad[no][3].x, quad[no][3].y, 1,          0,          0, 0, -quad[1-no][3].x * quad[no][3].x, -quad[1-no][3].x * quad[no][3].y,
               0,          0, 0, quad[no][3].x, quad[no][3].y, 1, -quad[1-no][3].y * quad[no][3].x, -quad[1-no][3].y * quad[no][3].y);
  m1 = m1.inv();
  m2 = (Mat_<float>(8, 1) <<
      quad[1-no][0].x, quad[1-no][0].y, quad[1-no][1].x, quad[1-no][1].y,
      quad[1-no][2].x, quad[1-no][2].y, quad[1-no][3].x, quad[1-no][3].y);

  Mat h = m1 * m2;
  Mat H = (Mat_<float>(3, 3) <<
      h.at<float>(0, 0), h.at<float>(1, 0), h.at<float>(2, 0),
      h.at<float>(3, 0), h.at<float>(4, 0), h.at<float>(5, 0),
      h.at<float>(6, 0), h.at<float>(7, 0), 1);
  return H;
}

double calTotalArea(int no){
  Point v1, v2, v3, v4;
  if(no == 0){
    v1 = calVector(quad1[0], quad1[1]);
    v2 = calVector(quad1[0], quad1[3]);
    v3 = calVector(quad1[2], quad1[1]);
    v4 = calVector(quad1[2], quad1[3]);
  }
  else{
    v1 = calVector(quad2[0], quad2[1]);
    v2 = calVector(quad2[0], quad2[3]);
    v3 = calVector(quad2[2], quad2[1]);
    v4 = calVector(quad2[2], quad2[3]);
  }
  return crossArea(v1, v2) + crossArea(v3, v4);
}

double calArea(Point p, int no){
  Point v1, v2, v3, v4;
  if(no == 0){
    v1 = calVector(p, quad1[0]);
    v2 = calVector(p, quad1[1]);
    v3 = calVector(p, quad1[2]);
    v4 = calVector(p, quad1[3]);
  }
  else{
    v1 = calVector(p, quad2[0]);
    v2 = calVector(p, quad2[1]);
    v3 = calVector(p, quad2[2]);
    v4 = calVector(p, quad2[3]);
  }
  return crossArea(v1, v2) + crossArea(v2, v3) + crossArea(v3, v4) + crossArea(v4, v1);
}
