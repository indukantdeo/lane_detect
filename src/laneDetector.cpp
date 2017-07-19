#include "../include/laneDetector_utils.hpp"
#include "../include/houghP.hpp"

using namespace cv;
using namespace std;

void on_low_h_thresh_trackbar(int, void *);
void on_high_h_thresh_trackbar(int, void *);
void on_low_s_thresh_trackbar(int, void *);
void on_high_s_thresh_trackbar(int, void *);
void on_low_v_thresh_trackbar(int, void *);
void on_high_v_thresh_trackbar(int, void *);

int low_h = 0, low_s= 0, low_v = 0;
int high_h = 179, high_s = 255, high_v = 255;

int lowThreshold=70;
int highThreshold=150;



int main()
{
	//Test HoughLinesP2
	/*Mat test_img=imread("images/test.png", 0);
	cv::resize(test_img, test_img, cv::Size(500,500));
	vector<Vec4i> test_lines;
	vector<int> test_lines_len;
	HoughLinesP2(test_img, 1, CV_PI/1800, 150, 80, 200, test_lines, test_lines_len, 10);
	cout<<test_lines.size()<<" "<<test_lines_len.size()<<endl;
	for(int o=0;o<test_lines.size();o++)
	{
		cout<<test_lines[o][0]<<" "<<test_lines[o][1]<<" "<<test_lines[o][2]<<" "<<test_lines[o][3]<<"-"<<test_lines_len[o]<<endl;
		circle(test_img, {test_lines[o][0], test_lines[o][1]}, 5, Scalar(255), 1, 8, 0);
		circle(test_img, {test_lines[o][2], test_lines[o][3]}, 5, Scalar(255), 1, 8, 0);
	}
	imshow("aaa", test_img);
	waitKey(0);
	*/

	
	//initialize important variables here 

	

	

       //VideoCapture cap("/home/indu/agv/road.mp4");
       //if(!cap.isOpened())  // check if we succeeded
           // return -1;
  
  //int f=1;//frame count
  
  //cv::Mat img1;
 //while(1)
 //{
  
 
  //cap.read(img1);
  //if(!(f++%10==0))
    //continue;
  //namedWindow("image",WINDOW_NORMAL);
  //imshow("image",img1);
 
  const int n_segments=5;
  int segments[n_segments]={75, 140, 215, 250, 320};
  Mat img_segments[5];
  std::stringstream window_name;

  cv::Mat img1=cv::imread("/home/indu/data_road/testing/image_2/uu_000076.png", 1);
 

  //string ty =  type2str(img.type());
  //printf("Matrix: %s %dx%d \n", ty.c_str(), img.cols, img.rows );
  
  cvtColor(img1, img1, CV_BGR2HSV);
	

  // color extraction
	
	Mat A = img1.clone();
	
	cout<<"Enter 1 for trackbar or enter 0 for predefined value";cout<<endl;
	int q;
	cin>>q;cout<<endl;	

	if(q==1)
	{
	namedWindow("Video Capture", WINDOW_NORMAL);
	namedWindow("Object Detection", WINDOW_NORMAL);
	createTrackbar("Low h", "Object Detection", &low_h, 179, on_low_h_thresh_trackbar);
	createTrackbar("High h", "Object Detection", &high_h, 179, on_high_h_thresh_trackbar);
	createTrackbar("Low s", "Object Detection", &low_s, 255, on_low_s_thresh_trackbar);
	createTrackbar("High s", "Object Detection", &high_s, 255, on_high_s_thresh_trackbar);
	createTrackbar("Low v", "Object Detection", &low_v, 255, on_low_v_thresh_trackbar);
	createTrackbar("High v", "Object Detection", &high_v, 255, on_high_v_thresh_trackbar);
	while ((char)waitKey(1) != 'q')
		{
			inRange(img1, Scalar(low_h, low_s, low_v), Scalar(high_h, high_s, high_v), A);	
			imshow("Object Detection",A);	
			imshow("Video Capture", img1);
		}
	cvtColor(A, img1, CV_GRAY2BGR);
	}

if(q==0)
 {
	//white			
	inRange(img1, Scalar(0, 0, 209), Scalar(105, 70, 255), A);			
	

	cvtColor(A, img1, CV_GRAY2BGR);
	
 }


  cvtColor(img1,img1,CV_BGR2GRAY);

  cv::Mat img=horizontal_gradient(img1);
  cvtColor(img, img, CV_GRAY2BGR);
	
	cv::resize(img, img, cv::Size(1000,1000));
	//imshow("lanes", img);
	


	//img=removeShadow(img);

	/*vary canny paramters
	int max_lowThreshold=500, max_highThreshold=500;
	namedWindow("Edge threshold", CV_WINDOW_AUTOSIZE);
	createTrackbar( "Min Threshold:", "Edge threshold", &lowThreshold, max_lowThreshold);
	createTrackbar( "Max Threshold:", "Edge threshold", &highThreshold, max_highThreshold);

	Mat edges;
	while(true)
	{
		edges=find_edges(img);
		imshow("edges", edges);

		char c=(char)waitKey(10);
		if(c=='q') 
			break;
	}*/

	//Mat edges=img.clone();
  //cvtColor(edges,edges,CV_HSV2BGR);
  //cvtColor(edges,edges,CV_BGR2GRAY);
  Mat edges=findEdges(img);
  
	extractSegments(img_segments, edges, segments, n_segments);

	int i, j, k;

	//display edge segments
	/*for(i=0;i<n_segments;i++)
		cout<<img_segments[i].rows<<" "<<img_segments[i].cols<<endl;
	
	for(i=0;i<n_segments;i++)
  	{
  		window_name<<"edges"<<i<<"";
  		imshow(window_name.str(), img_segments[i]);
  		window_name.str("");
  	}*/

	vector<Vec4i> lines[n_segments], lane_lines[n_segments];
	int hough_threshold[5]={30, 30, 40, 50, 50};
	int hough_minLineLength[5]={20, 25, 25, 30, 50};
	for(i=0; i<n_segments ;i++)
		HoughLinesP(img_segments[i], lines[i], 1, CV_PI/180, hough_threshold[i], hough_minLineLength[i], 50 );

  	Mat line_segments[n_segments];
  	Mat empty=img-img;
  	extractSegments(line_segments, empty, segments, n_segments);


  	for(i=0;i<n_segments;i++)
  		for(j=0;j<lines[i].size();j++)
  		{
  			Vec4i l = lines[i][j];
  			line( line_segments[i], Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
  		}


  	/* display line segments
  	for(i=0;i<n_segments;i++)
  	{
  		window_name<<"window"<<i<<"";
  		imshow(window_name.str(), line_segments[i]);
  		window_name.str("");
  	}*/


  	Mat line=img-img;
  	mergeSegments(line_segments, line, segments, n_segments);
  	imshow("detected lines",line);

  	for(i=0;i<n_segments;i++)
  		cout<<"#lines for "<<i<<": "<<lines[i].size()<<endl;

  	int vanish_row_vote[2000]={0};

  	int h=1000;		//h=height
  	for(i=4;i>=3;i--)
  	{
  		h-=segments[i];
  		for(j=0;j<lines[i].size();j++)
  			for(k=0;k<lines[i].size();k++)
  			{
  				if(j==k)
  					continue;

  				int vanish_row=findIntersection(lines[i][j], lines[i][k])+h;

  				//for checking intersection function
  				/*Mat ci(1000, 1000, CV_8UC3, Scalar(0));
  				Mat cs[5];
  				extract_segments(cs, ci, segments, n_segments);
  				cv::line( cs[i], Point(lines[i][j][0], lines[i][j][1]), Point(lines[i][j][2], lines[i][j][3]), Scalar(255,0,0), 3, CV_AA, 0);
  				cv::line( cs[i], Point(lines[i][k][0], lines[i][k][1]), Point(lines[i][k][2], lines[i][k][3]), Scalar(255,0,0), 3, CV_AA, 0);
  				merge_segments(cs, ci, segments, n_segments);
  				imshow("check", ci);

  				cout<<1000-vanish_row<<endl;
  				waitKey(2000);*/

  				if(vanish_row>-1000 && vanish_row<1000)
  					vanish_row_vote[1000-vanish_row]++;
  			}
  	}

  	int current_votes=0;
  	int max_votes=-1, max_i=-1;
  	for(i=0;i<50;i++)
  		current_votes+=vanish_row_vote[i];

  	for(i=50;i<2000;i++)
  	{
  		current_votes+=vanish_row_vote[i];
  		current_votes-=vanish_row_vote[i-50];

  		if(current_votes>=max_votes)
  		{
  			max_votes=current_votes;
  			max_i=i;
  		}
  	}

  	int vanish_row=max_i-25;

  	cout<<"Vanishing row: "<<vanish_row<<" with votes: "<<max_votes<<endl;

  	Mat output(1200, 1000, CV_8UC3, Scalar(0));
  	line.copyTo(output(cv::Rect(0, 200, 1000, 1000)));
  	cv::line( output, Point(0, 1200-vanish_row), Point(1000, 1200-vanish_row), Scalar(255,0,0), 10, CV_AA, 0);
  	cv::line( img, Point(0, 1000-vanish_row), Point(1000, 1000-vanish_row), Scalar(255,0,0), 10, CV_AA, 0);
  	//line( output, Point(0, 1000-vanish_row), Point(1000, 1000-vanish_row), Scalar(255,0,0), 10, CV_AA);


  	imshow("output", output);

  	Mat lanes(1000, 1000, CV_8UC3, Scalar(0)); 
  	Mat lanes_segments[n_segments];
  	extractSegments(lanes_segments, lanes, segments, n_segments);

  	int top_lane_segment=4;
  	h=0;
  	for(i=4;i>=2;i--)
  	{
  		for(j=0;j<lines[i].size();j++)
  			for(k=0;k<lines[i].size();k++)
  			{
  				if(j==k)
  					continue;

  				int vanishRow=(segments[i]-findIntersection(lines[i][j], lines[i][k]))+h;

  				if(vanishRow>= vanish_row-20 && vanishRow<= vanish_row+20)
  				{
  					top_lane_segment=i;
  					//warning: lane_lines has duplicates
  					lane_lines[i].push_back(lines[i][j]);
  					lane_lines[i].push_back(lines[i][k]);

  					cv::line( lanes_segments[i], Point(lines[i][j][0], lines[i][j][1]), Point(lines[i][j][2], lines[i][j][3]), Scalar(255,0,0), 3, CV_AA, 0);
  					cv::line( lanes_segments[i], Point(lines[i][k][0], lines[i][k][1]), Point(lines[i][k][2], lines[i][k][3]), Scalar(255,0,0), 3, CV_AA, 0);
  					/*imshow("please", lanes_segments[i]);

  					Mat temp(segments[i], 1000, CV_8UC3, Scalar(0));
  					cv::line( temp, Point(lines[i][j][0], lines[i][j][1]), Point(lines[i][j][2], lines[i][j][3]), Scalar(255,0,0), 3, CV_AA, 0);
  					cv::line( temp, Point(lines[i][k][0], lines[i][k][1]), Point(lines[i][k][2], lines[i][k][3]), Scalar(255,0,0), 3, CV_AA, 0);

  					imshow("verify", temp);

  					waitKey(2000);*/

  				}	
  			}
  		h+=segments[i];
  	}
  	mergeSegments(lanes_segments, lanes, segments, n_segments);

  	//remove duplicates from lane_lines
  	cout<<"Before removing duplicated: "; for(i=0;i<=4;i++) cout<<lane_lines[i].size()<<" "; cout<<endl;
  	for(i=0;i<n_segments;i++)
  	{
  		for(j=0;j<lane_lines[i].size();j++)
  			for(k=0;k<lane_lines[i].size();k++)
  			{
  				if(j>= lane_lines[i].size() || k>= lane_lines[i].size())
  					continue;

  				if(j==k)
  					continue;

  				if(lane_lines[i][j][0]==lane_lines[i][k][0] && lane_lines[i][j][1]==lane_lines[i][k][1] && lane_lines[i][j][2]==lane_lines[i][k][2] && lane_lines[i][j][3]==lane_lines[i][k][3])
  					lane_lines[i].erase(lane_lines[i].begin() + k);
  			}
  	}
  	cout<<"After removing duplicated: "; for(i=0;i<=4;i++) cout<<lane_lines[i].size()<<" "; cout<<endl;

  	/*vector<Vec4i> center_lane_lines[n_segments];
  	Point control_points[n_segments+1];
  	int num_control_points=1;
  	int lane_center=img.cols/2;
  	h=1000;
  	circle(img, {img.cols/2, img.rows}, 20, Scalar(0, 0, 255), 5, 8, 0);
  	control_points[n_segments]={img.cols/2, img.rows};
  	for(i=n_segments-1;i>=top_lane_segment;i--)
  	{
  		h-=segments[i];
  		if(1000-h>vanish_row) break;
  		control_points[i].y=h;
  		getCenterLanes(segments[i], img.cols, control_points[i], lane_lines[i], center_lane_lines[i], lane_center);
  		circle(img, control_points[i], 20, Scalar(0, 0, 255), 5, 8, 0);
  		num_control_points++;
  		cout<<control_points[i].x<<" "<<control_points[i].y<<endl;
  	}

  	drawBezierSpline(img, control_points, num_control_points, n_segments);
  	cout<<"spline drawn"<<endl;*/

  	for(i=1000-vanish_row;i>=0;i--)
  		for(j=0;j<img.cols;j++)
  			lanes.at<Vec3b>(i, j)={0, 0, 0};

  	for(i=0;i<img.rows;i++)
  		for(j=0;j<img.cols;j++)
  			if(lanes.at<Vec3b>(i, j)[0]==255)
  				img.at<Vec3b>(i, j)={255, 0, 0};

  	//visualizing image splits
  	h=0;
  	for(i=0;i<4;i++)
  	{
  		h+=segments[i];
  		cv::line( img, Point(0, h), Point(1000, h), Scalar(0,0,0), 5, CV_AA, 0);
  	}

  	imshow("original image", img);
  	imshow("lanes", lanes);
	  imshow("edges", edges);
    namedWindow("lane1",WINDOW_NORMAL);
    
    cvtColor(lanes,lanes,CV_BGR2GRAY);
    //cv::Mat trial=vertical_gradient(lanes);
    cv::Mat trial=horizontal_gradient(lanes);
    imshow("lane1",trial);
  //waitKey(1);

	waitKey(0);



	return 0;
}


void on_low_h_thresh_trackbar(int, void *)
{
  low_h = min(high_h - 1, low_h);
  setTrackbarPos("Low H", "Object Detection", low_h);
}
void on_high_h_thresh_trackbar(int, void *)
{
  high_h = max(high_h, low_h + 1);
  setTrackbarPos("High H", "Object Detection", high_h);
}
void on_low_s_thresh_trackbar(int, void *)
{
  low_s = min(high_s - 1, low_s);
  setTrackbarPos("Low S", "Object Detection", low_s);
}
void on_high_s_thresh_trackbar(int, void *)
{
  high_s = max(high_s, low_s + 1);
  setTrackbarPos("High S", "Object Detection", high_s);
}
void on_low_v_thresh_trackbar(int, void *)
{
  low_v = min(high_v - 1, low_v);
  setTrackbarPos("Low V", "Object Detection", low_v);
}
void on_high_v_thresh_trackbar(int, void *)
{
  high_v = max(high_v, low_v + 1);
  setTrackbarPos("High V", "Object Detection", high_v);
}



