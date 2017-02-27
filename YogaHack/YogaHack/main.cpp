// Body.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
// This source code is licensed under the MIT license. Please see the License in License.txt.
// "This is preliminary software and/or hardware and APIs are preliminary and subject to change."
//

#include "stdafx.h"
#include <Windows.h>
#include <Kinect.h>
#include <opencv2/opencv.hpp>


template<class Interface>
inline void SafeRelease( Interface *& pInterfaceToRelease )
{
	if( pInterfaceToRelease != NULL ){
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

int _tmain( int argc, _TCHAR* argv[] )
{
	cv::setUseOptimized( true );

	// Sensor
	IKinectSensor* pSensor;
	HRESULT hResult = S_OK;
	hResult = GetDefaultKinectSensor( &pSensor );
	if( FAILED( hResult ) ){
		std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
		return -1;
	}

	hResult = pSensor->Open( );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::Open()" << std::endl;
		return -1;
	}

	// Source
	IColorFrameSource* pColorSource;
	hResult = pSensor->get_ColorFrameSource( &pColorSource );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
		return -1;
	}

	IBodyIndexFrameSource* pBodyIndexSource;
	hResult = pSensor->get_BodyIndexFrameSource( &pBodyIndexSource );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_BodyIndexFrameSource()" << std::endl;
		return -1;
	}

	IBodyFrameSource* pBodySource;
	hResult = pSensor->get_BodyFrameSource( &pBodySource );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_BodyFrameSource()" << std::endl;
		return -1;
	}

	// Reader
	IColorFrameReader* pColorReader;
	hResult = pColorSource->OpenReader( &pColorReader );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	IBodyIndexFrameReader* pBodyIndexReader;
	hResult = pBodyIndexSource->OpenReader( &pBodyIndexReader );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IBodyIndexFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	IBodyFrameReader* pBodyReader;
	hResult = pBodySource->OpenReader( &pBodyReader );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IBodyFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	// Description
	IFrameDescription* pDescription;
	hResult = pColorSource->get_FrameDescription( &pDescription );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	}

	int width = 0;
	int height = 0;
	pDescription->get_Width( &width ); // 1920
	pDescription->get_Height( &height ); // 1080
	unsigned int bufferSize = width * height * 4 * sizeof( unsigned char );

	cv::Mat bufferMat( height, width, CV_8UC4 );
	cv::Mat bufferjointMat( height, width, CV_8UC4 );
	cv::Mat bodyMat( height / 2, width / 2, CV_8UC4 );
	cv::Mat jointMat( height / 2, width / 2, CV_8UC4 );
	cv::Mat shadowMat( 424, 512, CV_8UC3 );
	cv::Mat leanMat( 400, 400, CV_8UC3, cv::Scalar(0));
	cv::namedWindow( "Color" );
	cv::namedWindow( "Body" );
	cv::namedWindow( "Joint" );
	cv::namedWindow("Lean");

	// Color Table
	cv::Vec3b color[BODY_COUNT];
	color[0] = cv::Vec3b( 255,   0,   0 );
	color[1] = cv::Vec3b(   0, 255,   0 );
	color[2] = cv::Vec3b(   0,   0, 255 );
	color[3] = cv::Vec3b( 255, 255,   0 );
	color[4] = cv::Vec3b( 255,   0, 255 );
	color[5] = cv::Vec3b(   0, 255, 255 );

	// Coordinate Mapper
	ICoordinateMapper* pCoordinateMapper;
	hResult = pSensor->get_CoordinateMapper( &pCoordinateMapper );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << std::endl;
		return -1;
	}

	while( 1 ){
		// Frame
		IColorFrame* pColorFrame = nullptr;
		hResult = pColorReader->AcquireLatestFrame( &pColorFrame );
		if( SUCCEEDED( hResult ) ){
			hResult = pColorFrame->CopyConvertedFrameDataToArray( bufferSize, reinterpret_cast<BYTE*>( bufferMat.data ), ColorImageFormat::ColorImageFormat_Bgra );
			if( SUCCEEDED( hResult ) ){
				cv::resize( bufferMat, bodyMat, cv::Size(), 0.5, 0.5 );
			}
		}


		IBodyIndexFrame* pBodyIndexFrame = nullptr;
		hResult = pBodyIndexReader->AcquireLatestFrame( &pBodyIndexFrame );
		if( SUCCEEDED( hResult ) ){
			unsigned int bufferSize = 0;
			unsigned char* buffer = nullptr;
			hResult = pBodyIndexFrame->AccessUnderlyingBuffer( &bufferSize, &buffer );
			if( SUCCEEDED( hResult ) ){
				for( int y = 0; y < 424; y++ ){
					for( int x = 0; x < 512; x++ ){
						unsigned int index = y * 512 + x;
						if( buffer[index] != 0xff ){
							shadowMat.at<cv::Vec3b>( y, x ) = color[buffer[index]];
						}
						else{
							shadowMat.at<cv::Vec3b>( y, x ) = cv::Vec3b( 0, 0, 0 );
						}
					}
				}
			}
		}


		cv::rectangle(bufferjointMat, cv::Point(0,0), cv::Point(width, height), cv::Scalar(0,0,0), -1, CV_AA);

		IBodyFrame* pBodyFrame = nullptr;
		hResult = pBodyReader->AcquireLatestFrame( &pBodyFrame );
		if( SUCCEEDED( hResult ) ){
			IBody* pBody[BODY_COUNT] = { 0 };
			hResult = pBodyFrame->GetAndRefreshBodyData( BODY_COUNT, pBody );
			if( SUCCEEDED( hResult ) ){
				for( int count = 0; count < BODY_COUNT; count++ ){
					BOOLEAN bTracked = false;
					hResult = pBody[count]->get_IsTracked( &bTracked );
					if( SUCCEEDED( hResult ) && bTracked ){
						Joint joint[JointType::JointType_Count];
						hResult = pBody[ count ]->GetJoints( JointType::JointType_Count, joint );
						if( SUCCEEDED( hResult ) ){
							// Left Hand State
							HandState leftHandState = HandState::HandState_Unknown;
							hResult = pBody[count]->get_HandLeftState( &leftHandState );
							if( SUCCEEDED( hResult ) ){
								std::cout << count << "の左手をGETしています" << std::endl;
								ColorSpacePoint colorSpacePoint = { 0 };
								hResult = pCoordinateMapper->MapCameraPointToColorSpace( joint[JointType::JointType_HandLeft].Position, &colorSpacePoint );
								if( SUCCEEDED( hResult ) ){
									int x = static_cast<int>( colorSpacePoint.X );
									int y = static_cast<int>( colorSpacePoint.Y );
									
									if( ( x > 50 ) && ( x < (width - 50) ) && ( y > 50 ) && ( y < (height - 50) ) ){
										if( leftHandState == HandState::HandState_Open ){
											cv::circle( bufferjointMat, cv::Point( x, y ), 50, cv::Scalar( 0, 128, 0 ), 5, CV_AA );
										}
										else if( leftHandState == HandState::HandState_Closed ){
											cv::circle( bufferjointMat, cv::Point( x, y ), 50, cv::Scalar( 0, 0, 128 ), 5, CV_AA );
										}
										else if( leftHandState == HandState::HandState_Lasso ){
											cv::circle( bufferjointMat, cv::Point( x, y ), 50, cv::Scalar( 128, 128, 0 ), 5, CV_AA );
										}
									}
									
									std::cout << count << "のLeftHandX : " << x << ", " << count << "のLeftHandY : " << y << std::endl;
								}
							}

							// Right Hand State
							HandState rightHandState = HandState::HandState_Unknown;
							hResult = pBody[count]->get_HandRightState( &rightHandState );
							if( SUCCEEDED( hResult ) ){
								ColorSpacePoint colorSpacePoint = { 0 };
								hResult = pCoordinateMapper->MapCameraPointToColorSpace( joint[JointType::JointType_HandRight].Position, &colorSpacePoint );
								if( SUCCEEDED( hResult ) ){
									int x = static_cast<int>( colorSpacePoint.X );
									int y = static_cast<int>( colorSpacePoint.Y );
									
									if( ( x > 60 ) && ( x < (width - 60) ) && ( y > 60 ) && ( y < (height - 60) ) ){
										if( rightHandState == HandState::HandState_Open ){
											cv::circle( bufferjointMat, cv::Point( x, y ), 50, cv::Scalar( 0, 128, 0 ), 5, CV_AA );
										}
										else if( rightHandState == HandState::HandState_Closed ){
											cv::circle( bufferjointMat, cv::Point( x, y ), 50, cv::Scalar( 0, 0, 128 ), 5, CV_AA );
										}
										else if( rightHandState == HandState::HandState_Lasso ){
											cv::circle( bufferjointMat, cv::Point( x, y ), 50, cv::Scalar( 128, 128, 0 ), 5, CV_AA );
										}
									}
									
									std::cout << count << "のRightHandX : " << x << ", " << count << "のRightHandY : " << y << std::endl;
								}
							}

							/*
							// Joint
							for( int type = 0; type < JointType::JointType_Count; type++ ){
								ColorSpacePoint colorSpacePoint = { 0 };
								pCoordinateMapper->MapCameraPointToColorSpace( joint[type].Position, &colorSpacePoint );
								int x = static_cast<int>( colorSpacePoint.X );
								int y = static_cast<int>( colorSpacePoint.Y );
								if( ( x > 10 ) && ( x < (width - 10) ) && ( y > 10 ) && ( y < (height - 10) ) ){
									cv::circle( bufferjointMat, cv::Point( x, y ), 10, static_cast< cv::Scalar >( color[count] ), -1, CV_AA );
								}
							}
							*/
						}
						/*
						// Activity
						UINT capacity = 0;
						DetectionResult detectionResults = DetectionResult::DetectionResult_Unknown;
						hResult = pBody[count]->GetActivityDetectionResults( capacity, &detectionResults );
						if( SUCCEEDED( hResult ) ){
							if( detectionResults == DetectionResult::DetectionResult_Yes ){
								switch( capacity ){
									case Activity::Activity_EyeLeftClosed:
										std::cout << "Activity_EyeLeftClosed" << std::endl;
										break;
									case Activity::Activity_EyeRightClosed:
										std::cout << "Activity_EyeRightClosed" << std::endl;
										break;
									case Activity::Activity_MouthOpen:
										std::cout << "Activity_MouthOpen" << std::endl;
										break;
									case Activity::Activity_MouthMoved:
										std::cout << "Activity_MouthMoved" << std::endl;
										break;
									case Activity::Activity_LookingAway:
										std::cout << "Activity_LookingAway" << std::endl;
										break;
									default:
										break;
								}
							}
						}
						else{
							std::cerr << "Error : IBody::GetActivityDetectionResults()" << std::endl;
						}
						*/
						/*
						// Appearance
						capacity = 0;
						detectionResults = DetectionResult::DetectionResult_Unknown;
						hResult = pBody[count]->GetAppearanceDetectionResults( capacity, &detectionResults );
						if( SUCCEEDED( hResult ) ){
							if( detectionResults == DetectionResult::DetectionResult_Yes ){
								switch( capacity ){
									case Appearance::Appearance_WearingGlasses:
										std::cout << "Appearance_WearingGlasses" << std::endl;
										break;
									default:
										break;
								}
							}
						}
						else{
							std::cerr << "Error : IBody::GetAppearanceDetectionResults()" << std::endl;
						}

						// Expression
						capacity = 0;
						detectionResults = DetectionResult::DetectionResult_Unknown;
						hResult = pBody[count]->GetExpressionDetectionResults( capacity, &detectionResults );
						if( SUCCEEDED( hResult ) ){
							if( detectionResults == DetectionResult::DetectionResult_Yes ){
								switch( capacity ){
									case Expression::Expression_Happy:
										std::cout << "Expression_Happy" << std::endl;
										break;
									case Expression::Expression_Neutral:
										std::cout << "Expression_Neutral" << std::endl;
										break;
									default:
										break;
								}
							}
						}
						else{
							std::cerr << "Error : IBody::GetExpressionDetectionResults()" << std::endl;
						}
						*/
						// Lean
						PointF amount;
						hResult = pBody[count]->get_Lean( &amount );
						if( SUCCEEDED( hResult ) ){
							std::cout << "LeanX : " << amount.X << ", LeanY: " << amount.Y << std::endl;
							cv::rectangle(leanMat, cv::Point(0,0), cv::Point(400, 400), cv::Scalar(0,0,0), -1, CV_AA);
							cv::RotatedRect rRect = cv::RotatedRect(cv::Point2f(200, 200), cv::Size2f(20,150), amount.Y*-45);
							cv::Point2f vertices[4];
							rRect.points(vertices);
							for(int v = 0; v < 4; v++){
								cv::line(leanMat, vertices[v], vertices[(v+1)%4], static_cast< cv::Scalar >( color[count] ));
							}
						}
					}
				}
				cv::resize( bufferjointMat, jointMat, cv::Size(), 0.5, 0.5 );
			}
			for( int count = 0; count < BODY_COUNT; count++ ){
				SafeRelease( pBody[count] );
			}
		}

		SafeRelease( pColorFrame );
		SafeRelease( pBodyFrame );
		SafeRelease( pBodyIndexFrame );

		cv::imshow( "Color", bodyMat );
		cv::imshow( "Joint", jointMat );
		cv::imshow("Body", shadowMat);
		cv::imshow("Lean", leanMat);

		if( cv::waitKey( 10 ) == VK_ESCAPE ){
			break;
		}
	}

	SafeRelease( pColorSource );
	SafeRelease( pBodySource );
	SafeRelease( pColorReader );
	SafeRelease( pBodyReader );
	SafeRelease( pDescription );
	SafeRelease( pCoordinateMapper );
	if( pSensor ){
		pSensor->Close();
	}
	SafeRelease( pSensor );
	cv::destroyAllWindows();

	return 0;
}

