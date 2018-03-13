#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main(int argc, char* argv[]){

  string file[4] = {"Bunny", "Eagle", "Teapot", "Venus"};
  char data[50];
  float para[105];
  int count = 0;

  FILE *parmeter = fopen("./Bunny/camera parmeter.txt.txt", "r");

  //read camera parameter file
  while(fgets(data, 50, parmeter) != NULL){

    if(data[0] == '#'){
      if(count < 9){
        for(int i = 0; i < 9; i++){
          fscanf(parmeter, "%f", &para[count++]);
        }
      }
      else{
        for(int i = 0; i < 12; i++){
          fscanf(parmeter, "%f", &para[count++]);
        }
      }
    }
  }
  fclose(parmeter);

  count = 0;
  vector<Mat> Rt;
  Mat K(3, 3, CV_32F);

  //set intrinsic parameter
  for(int i = 0; i < 3; i++){
    for(int j = 0; j < 3; j++){
      K.at<float>(i, j) = para[count++];
    }
  }

  //set extrinsic parameter
  for(int i = 0; i < 8; i++){
    Mat rt(3, 4, CV_32F);
    for(int j = 0; j < 3; j++){
      for(int k = 0; k < 4; k++){
        rt.at<float>(j, k) = para[count++];
      }
    }
    Rt.push_back(rt);
  }

  ////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////

  char filename[20];
  stringstream ss;

  for(int i = 0; i < 4; i++){  // 4 folder
    ss.str("");
    ss.clear();
    ss << "./"<< file[i] << ".xyz";
    ss >> filename;
    FILE *output = fopen(filename, "w");
    vector<Mat> src;

    for(int j = 1; j < 9; j++){ //read 8 source image
      ss.str("");
      ss.clear();
      ss << "./" << file[i] << "/00" << j << ".bmp";
      ss >> filename;
      Mat img;
      img = imread(filename, 0);
      src.push_back(img);
    }

    Mat wPos(4, 1, CV_32F);
    Mat imgPos(3, 1, CV_32F);
    bool background = false;
    int width = src[0].cols, height = src[0].rows;

    //object range in world coordinate
    for(int x = -50; x < 50; x++){
      for(int y = -50; y < 50; y++){
        for(int z = -10; z < 90; z++){
          wPos = (Mat_<float>(4, 1) << x, y, z, 1);
          background = false;
          //check the position(world coordinate) in image is black or white
          for(int j = 0; j < 8; j++){
            imgPos = K * Rt[j] * wPos;
            //normalize
            int imgX = imgPos.at<float>(0, 0) / imgPos.at<float>(2, 0);
            int imgY = imgPos.at<float>(1, 0) / imgPos.at<float>(2, 0);
            //the world coordinate point out of range or is black(background)
            if(imgX < 0 || imgY < 0 || imgY >= height || imgX >= width || src[j].at<uchar>(imgY, imgX) == 0){
              background = true;
              break;
            }
          }
          //write foreground point into file
          if(!background) fprintf(output, "%d %d %d\n", x, y, z);
        }
      }
    }
    fclose(output);
  }
  return 0;
}
