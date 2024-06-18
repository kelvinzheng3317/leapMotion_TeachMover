#include "Kinect.h"

class MainPage {
  void InitKinect()
  {
    KinectSensor* sensor = KinectSensor::GetDefault();
    sensor->Open();
    bodyReader = sensor->BodyFrameSource->OpenReader();
    bodyReader->FrameArrived += 
    ref new TypedEventHandler<typename BodyFrameArrivedEventArgs^> (this,                 
                          &MainPage::OnBodyFrameArrived);
    bodies = ref new Platform::Collections::Vector<Body^>(6);
  }

  void OnBodyFrameArrived(BodyFrameReader ^sender, BodyFrameArrivedEventArgs ^eventArgs){
    BodyFrame* frame = eventArgs->FrameReference->AcquireFrame();
    if (frame != nullptr){
        frame->GetAndRefreshBodyData(bodies);
    }
  }
};
