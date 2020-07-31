#include <cstdio>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <vector>
#include <math.h>


#include <astra/astra.hpp>
#include <astra/streams/Image.hpp>


#include "XnAlgorithms.h"

class BodyFrameListener : public astra::FrameListener
{
public:
    BodyFrameListener(int maxFramesToProcess)
            : maxFramesToProcess_(maxFramesToProcess) {
    }
    bool is_finished() const { return isFinished_; }

    std::vector<std::vector<uint8_t>> rData;
    std::vector<std::vector<uint8_t>> gData;
    std::vector<std::vector<uint8_t>> bData;
    std::vector<std::vector<astra::Vector2f>> bodyData;
private:
    void on_frame_ready(astra::StreamReader& reader,
                        astra::Frame& frame) override
    {
        const astra::ColorFrame colorFrame = frame.get<astra::ColorFrame>();
        const astra::DepthFrame depthFrame = frame.get<astra::DepthFrame>();
        const astra::BodyFrame bodyFrame = frame.get<astra::BodyFrame>();
        if (colorFrame.is_valid() && depthFrame.is_valid() && bodyFrame.is_valid())
        {
            store_color_frame(colorFrame);
            store_body_frame(bodyFrame);
            ++framesProcessed_;
        }

        isFinished_ = framesProcessed_ >= maxFramesToProcess_;
    }
    /*store rgb data into three vectors of int then store it into rData, gData, bData member variables*/
    bool store_color_frame(const astra::ColorFrame& colorFrame)
    {
        astra::RgbPixel tmp;
        std::vector<uint8_t> tmpVecR;
        std::vector<uint8_t> tmpVecG;
        std::vector<uint8_t> tmpVecB;


        for (int i = 0; i < colorFrame.height(); i++) {
            for (int j = 0; j < colorFrame.width(); j++) {
                tmp = colorFrame.data()[(j+i*colorFrame.width())];
                tmpVecR.push_back(tmp.r);
                tmpVecG.push_back(tmp.g);
                tmpVecB.push_back(tmp.b);

            }
        }
        rData.push_back(tmpVecR);
        gData.push_back(tmpVecG);
        bData.push_back(tmpVecB);

        return true;
    }
    /*store projected x,y position from bodyFrame into a Vector of 2D Vectors*/
    bool store_body_frame(const astra::BodyFrame& bodyFrame)
    {
        std::vector<astra::Vector2f> tmpBody;
        std::cout << bodyFrame.bodies()[0].joints()[0].depth_position().x << " " << bodyFrame.bodies()[0].joints()[0].depth_position().y << std::endl;

        const float jointScale = bodyFrame.info().width() / 120.f;
        astra::Body body = bodyFrame.bodies()[0];

        const auto& joints = body.joints();

        for (const auto& joint : joints) {
            astra::JointType type = joint.type();
            //projected x, y position
            const auto &pos = joint.depth_position();
            tmpBody.push_back(pos);
        }

        bodyData.push_back(tmpBody);
        return true;
        //int b = bodyFrame.bodies()[0].;
        //std::cout << std::to_string(b) << std::endl;
        //const u_int8_t* da = bm.data();

        //tmpBody.push_back(da[(j+i*colorFrame.width())]);
        //tmpBody.push_back(b);

        //bodyData.push_back(tmpBody);
    }


    bool isFinished_{false};
    int framesProcessed_{0};
    int maxFramesToProcess_{0};
};

void generate_color_file(std::vector<std::vector<uint8_t>> rData, std::vector<std::vector<uint8_t>> gData, std::vector<std::vector<uint8_t>> bData) {
    std::fstream rgbFile;
    std::string rgbFilename;

    std::vector<uint8_t> tmpVecR;
    std::vector<uint8_t> tmpVecG;
    std::vector<uint8_t> tmpVecB;

    for (int i = 0; i < rData.size(); i++) {
        tmpVecR = rData.at(i);
        tmpVecG = gData.at(i);
        tmpVecB = bData.at(i);

        rgbFilename = "rgb" + std::to_string(i);
        rgbFilename += ".ppm";
        rgbFile.open(rgbFilename, std::fstream::out);

        rgbFile << "P3" << std::endl;
        rgbFile << std::to_string(640);

        rgbFile << " " << 480 << std::endl << "255" << std::endl;

        for (int j = 0; j < tmpVecR.size(); j++) {
            //write color values into file ppm image
            rgbFile << std::to_string(tmpVecR.at(j)) << " " << std::to_string(tmpVecG.at(j)) << " "
                    << std::to_string(tmpVecB.at(j)) << " " << std::endl;
        }

        rgbFile.close();

    }
}

void generate_body_file(std::vector<std::vector<astra::Vector2f>> bodyData){

    std::fstream bodyFile;
    std::string bodyFilename;
    std::string tmp = "";
    std::string output = "";
    //int isKeyFrame = 0;
    std::vector<astra::Vector2f> tmpVecBody;
    bool isHead = false;

    for (int i = 0; i < bodyData.size(); i++){
        tmpVecBody = bodyData.at(i);
        //isKeyFrame = false;

        bodyFilename = "rgb" + std::to_string(i) + "_keypoints" ;
        bodyFilename += ".json";

        //bodyFile << "P3" << std::endl;
        //bodyFile << std::to_string(640);
        //bodyFile << " " << 480 << std::endl << "255" << std::endl;
        /*for (int k = 0; k < tmpVecBody.size(); k++){
            if (k < tmpVecBody.size()-1){
                tmp += std::to_string(tmpVecBody[k].x) + "," + std::to_string(tmpVecBody[k].y) + "," + "0.845918" + ",";
            }
            if(k == (tmpVecBody.size()-1)){
                tmp += std::to_string(tmpVecBody[k].x) + "," + std::to_string(tmpVecBody[k].y) + "," + "0.845918";
            }


        }*/
        //0
        tmp += std::to_string(tmpVecBody[0].x) + "," + std::to_string(tmpVecBody[0].y) + "," + "0.845918" + ",";
        //1
        tmp += std::to_string(tmpVecBody[1].x) + "," + std::to_string(tmpVecBody[1].y) + "," + "0.845918" + ",";
        //2
        tmp += std::to_string(tmpVecBody[2].x) + "," + std::to_string(tmpVecBody[2].y) + "," + "0.845918" + ",";
        //3
        tmp += std::to_string(tmpVecBody[3].x) + "," + std::to_string(tmpVecBody[3].y) + "," + "0.845918" + ",";
        //4
        tmp += std::to_string(tmpVecBody[16].x) + "," + std::to_string(tmpVecBody[16].y) + "," + "0.845918" + ",";
        //5
        tmp += std::to_string(tmpVecBody[5].x) + "," + std::to_string(tmpVecBody[5].y) + "," + "0.845918" + ",";
        //6
        tmp += std::to_string(tmpVecBody[6].x) + "," + std::to_string(tmpVecBody[6].y) + "," + "0.845918" + ",";
        //7
        tmp += std::to_string(tmpVecBody[17].x) + "," + std::to_string(tmpVecBody[17].y) + "," + "0.845918" + ",";
        //8
        tmp += std::to_string(tmpVecBody[9].x) + "," + std::to_string(tmpVecBody[9].y) + "," + "0.845918" + ",";
        //9
        tmp += std::to_string(tmpVecBody[10].x) + "," + std::to_string(tmpVecBody[10].y) + "," + "0.845918" + ",";
        //10
        tmp += std::to_string(tmpVecBody[11].x) + "," + std::to_string(tmpVecBody[11].y) + "," + "0.845918" + ",";
        //11
        tmp += std::to_string(tmpVecBody[12].x) + "," + std::to_string(tmpVecBody[12].y) + "," + "0.845918" + ",";
        //12
        tmp += std::to_string(tmpVecBody[13].x) + "," + std::to_string(tmpVecBody[13].y) + "," + "0.845918" + ",";
        //13
        tmp += std::to_string(tmpVecBody[14].x) + "," + std::to_string(tmpVecBody[14].y) + "," + "0.845918" + ",";
        //14
        tmp += std::to_string(tmpVecBody[15].x) + "," + std::to_string(tmpVecBody[15].y) + "," + "0.845918" + ",";
        //15
        tmp += "0.000000,0.000000,0.000000";
        //16
        tmp += ",0.000000,0.000000,0.000000";
        //17
        tmp += ",0.000000,0.000000,0.000000";
        //18
        tmp += ",0.000000,0.000000,0.000000";
        //19
        tmp += ",0.000000,0.000000,0.000000";
        //20
        tmp += ",0.000000,0.000000,0.000000";
        //21
        tmp += ",0.000000,0.000000,0.000000";
        //22
        tmp += ",0.000000,0.000000,0.000000";
        //23
        tmp += ",0.000000,0.000000,0.000000";
        //24
        tmp += ",0.000000,0.000000,0.000000";

        output = "{\"version\":1.3,\"people\":[{\"person_id\":[-1],\"pose_keypoints_2d\":[" + tmp +"],\"face_keypoints_2d\":[],\"hand_left_keypoints_2d\":[],\"hand_right_keypoints_2d\":[],\"pose_keypoints_3d\":[],\"face_keypoints_3d\":[],\"hand_left_keypoints_3d\":[],\"hand_right_keypoints_3d\":[]}]}";
        tmp = "";
        bodyFile.open(bodyFilename, std::fstream::out);
        bodyFile << output << std::endl;
        bodyFile.close();
        /*for (int j = 0; j < 640*480; j++) {
            //write color values into file ppm image
            for (int k = 0; k < tmpVecBody.size(); k++){
                if((floor(tmpVecBody[k].x)*floor(tmpVecBody[k].y)) == j && (floor(tmpVecBody[k].x)*floor(tmpVecBody[k].y)) != 0){
                    isKeyFrame += 5;
                    if (k == 0) isHead = true;
                    break;
                }
            }

            if(isKeyFrame && isHead) {
                bodyFile << 255 << " " << 0 << " " << 0 << " " << std::endl;
                isKeyFrame--;
            } else if (isKeyFrame){
                bodyFile << 0 << " " << 0 << " " << 0 << " " << std::endl;
                isKeyFrame--;
            }else{
                bodyFile << 255 << " " << 255 << " " << 255 << " " << std::endl;
            }
            //isKeyFrame = false;
            isHead = false;
        }*/
        //bodyFile.close();
    }
}

int main(int argc, char** argv)
{
    std::string tmp;

    astra::terminate();
    astra::initialize();
    orbbec_body_tracking_set_license("xyz");

    astra::StreamSet streamSet;
    astra::StreamReader reader = streamSet.create_reader();
    reader.stream<astra::DepthStream>().start();
    reader.stream<astra::ColorStream>().start();
    reader.stream<astra::BodyStream>().start();

    astra::ImageStreamMode imgMode;
    imgMode.set_fps(30);

    const int maxFramesToProcess = 600;
    BodyFrameListener listener(maxFramesToProcess);

    reader.add_listener(listener);

    do {
        astra_update();
    } while (!listener.is_finished());

    generate_color_file(listener.rData, listener.gData, listener.bData);
    generate_body_file(listener.bodyData);

    std::cout << "Press any key to continue...";
    std::cin.get();
    astra::terminate();
    std::cout << "hit enter to exit program" << std::endl;
    std::cin.get();
    return 0;
}
