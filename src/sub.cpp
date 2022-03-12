 
 //run camera
 // cout <<"ROBOT_ADJUST_TH"<<robot_adjust_th<<endl;
        // for(int i=x_pos-size;i<x_pos+size;i++)
        // {
        //     if(i<0||i>=mod_frame.rows)
        //     {
        //         continue;
        //     }
        //     else
        //     {
        //         if(y_pos+size>mod_frame.cols)
        //         {
        //             y_pos=mod_frame.cols-size-1;
        //         }
        //         if(y_pos-size<0)
        //         {
        //             y_pos=size;
        //         }
        //         frame.at<cv::Vec3b>(i,y_pos+size)={0,255,0};
        //         frame.at<cv::Vec3b>(i,y_pos-size)={0,255,0};
                
        //     }
        // }
        // for(int i=y_pos-size;i<y_pos+size;i++)
        // {
        //     if(i<0||i>=mod_frame.cols)
        //     {
        //         continue;
        //     }
        //     else
        //     {
        //         if(x_pos+size>mod_frame.rows)
        //         {
        //             x_pos=mod_frame.rows-size-1;
        //         }
        //         if(x_pos-size<0)
        //         {
        //             x_pos=size;
        //         }
        //         frame.at<cv::Vec3b>(x_pos+size,i)={0,255,0};
        //         frame.at<cv::Vec3b>(x_pos-size,i)={0,255,0};
        //     }
        // }