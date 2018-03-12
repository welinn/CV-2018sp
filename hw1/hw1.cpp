#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main(int argc, char* argv[]){

  string file[4] = {"./Bunny/", "./Eagle/", "./Teapot/", "./Venus/"};
  char data[50];
  float para[105];
  int count = 0;

  FILE *parmeter = fopen("./Bunny/camera parmeter.txt.txt", "r");

  while(fgets(data, 50, parmeter) != NULL){
    /*
    if((data[0] >= '0' && data[0] <= '9') || data[0] == '-'){
      printf("%s\n", data);
      int len = strlen(data);
      fseek(parmeter, -len, SEEK_CUR);
    }*/
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
  for(int i = 0; i < 3; i++){
    for(int j = 0; j < 3; j++){
      K.at<float>(i, j) = para[count++];
    }
  }

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
    ss << file[i] << "output.xyz";
    ss >> filename;
    FILE *output = fopen(filename, "w");
    vector<Mat> src;

    for(int j = 1; j < 9; j++){ // 8 source image
      ss.str("");
      ss.clear();
      ss << file[i] << "00" << j << ".bmp";
      ss >> filename;
      Mat img;
      img = imread(filename);
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
          wPos.at<float>(0, 0) = x;
          wPos.at<float>(1, 0) = y;
          wPos.at<float>(2, 0) = z;
          wPos.at<float>(3, 0) = 1;

          //check the position(world coordinate) in image is black or white
          //TODO pos wrong
          background = false;
          for(int j = 0; j < 8; j++){
            imgPos = K * Rt[j] * wPos;
            int imgX = imgPos.at<float>(0, 0) / imgPos.at<float>(2, 0);
            int imgY = imgPos.at<float>(1, 0) / imgPos.at<float>(2, 0);

            if(imgX >= 0 && imgY >= 0 && imgX < height && imgY < width && src[j].at<uchar>(imgX, imgY) != 0){
            //if(imgX < 0 || imgY < 0 || imgX >= height || imgY >= width || src[j].at<uchar>(imgX, imgY) == 0){
              background = true;
              break;
            }
          }
          if(!background){
            //is foreground, write the point into file
            fprintf(output, "%d %d %d\n", x, y, z);

          }
        }
      }
    }
    fclose(output);
  }

  return 0;
}
