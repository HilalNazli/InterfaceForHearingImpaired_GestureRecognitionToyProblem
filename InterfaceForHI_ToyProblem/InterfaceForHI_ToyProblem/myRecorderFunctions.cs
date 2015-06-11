using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Microsoft.Kinect;
using System.Windows.Controls;
using System.Threading;

namespace InterfaceForHI_ToyProblem
{
   public partial class MainWindow : Window, INotifyPropertyChanged
    {

        private const int MapDepthToByte = 8000 / 256;

        private void Reader_RecordMultiSourceFrameArrived(Object sender, MultiSourceFrameArrivedEventArgs e)
        {
            double[] bodyFeaturesNull = new double[2] { 0, 0 };
            bodyFeaturesSequence[nFrames] = bodyFeaturesNull;
          
            System.Console.WriteLine("nFrame:" + nFrames);

            MultiSourceFrameReference multiSourceFrameReference = e.FrameReference;
            MultiSourceFrame multiSourceFrame = multiSourceFrameReference.AcquireFrame();

            if (multiSourceFrame == null) return;

            ColorFrameReference colorFrameReference = multiSourceFrame.ColorFrameReference;
            //DepthFrameReference depthFrameReference = multiSourceFrame.DepthFrameReference;

            //DepthFrame depthFrame = depthFrameReference.AcquireFrame();
            ColorFrame colorFrame = colorFrameReference.AcquireFrame();
            if (colorFrameReference == null || colorFrame == null) return;
            //if (depthFrameReference == null || depthFrame == null) return;


            BodyFrameReference bodyFrameReference = multiSourceFrame.BodyFrameReference;
            BodyFrame bodyFrame = bodyFrameReference.AcquireFrame();

            if (bodyFrameReference == null || bodyFrame == null)
            {
               
            }
            // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
            // As long as those body objects are not disposed and not set to null in the array,
            // those body objects will be re-used.
            else
            {
                //handle null
                bodyFrame.GetAndRefreshBodyData(this.bodies);
                foreach (Body body in this.bodies)
                {
                    if (body.IsTracked)
                    {
                        double[] bodyFeatures = getBodyFeatures(body);
                        bodyFeaturesSequence[nFrames] = bodyFeatures;
                    }
                }
                bodyFrame.Dispose();
                nFrames = nFrames + 1;

            }

            
            // Start Showing vıdeo
            #region recordColorFrame

            FrameDescription colorFrameDescription = colorFrame.FrameDescription;
            if (colorFrame.RawColorImageFormat == ColorImageFormat.Bgra)
            {
                colorFrame.CopyRawFrameDataToArray(this.colorImage);
            }
            else
            {
                colorFrame.CopyConvertedFrameDataToArray(this.colorImage, ColorImageFormat.Bgra);
            }

            this.colorImageBitmap.WritePixels(
                new Int32Rect(0, 0, colorFrameDescription.Width, colorFrameDescription.Height),
                this.colorImage,
                colorFrameDescription.Width * this.bytesPerPixel,
                0);
            

            #endregion 
            colorFrame.Dispose();
             
        }


        private double[]  getBodyFeatures(Body myBody)
        {

            IReadOnlyDictionary<JointType, Joint> myJoints = myBody.Joints;

            // Coordinates
            double[] positionXYZ_rightandleft = new double[2];
            //positionXYZ_rightandleft[0] = myJoints[JointType.HandLeft].Position.X;
            //positionXYZ_rightandleft[1] = myJoints[JointType.HandLeft].Position.Y;
            //positionXYZ_rightandleft[2] = myJoints[JointType.HandLeft].Position.Z;
            positionXYZ_rightandleft[0] = myJoints[JointType.HandRight].Position.X;
            positionXYZ_rightandleft[1] = myJoints[JointType.HandRight].Position.Y;
            //positionXYZ_rightandleft[5] = myJoints[JointType.HandRight].Position.Z;

            
            return positionXYZ_rightandleft;
           
        }
    }





} 

