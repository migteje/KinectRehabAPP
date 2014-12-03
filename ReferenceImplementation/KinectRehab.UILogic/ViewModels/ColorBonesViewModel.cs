
namespace KinectRehab.UILogic.ViewModels
{
    using Microsoft.Kinect.Toolkit.Input;
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.Linq;
    using System.Runtime.CompilerServices;
    using System.Runtime.InteropServices.WindowsRuntime;
    using System.Threading.Tasks;
    using Microsoft.Practices.Prism.StoreApps;
    using Microsoft.Practices.Prism.StoreApps.Interfaces;
    using Windows.Foundation;
    using Windows.Graphics.Imaging;
    using Windows.UI;
    using Windows.UI.Popups;
    using Windows.UI.Xaml;
    using Windows.UI.Xaml.Controls;
    using Windows.UI.Xaml.Input;
    using Windows.UI.Xaml.Media;
    using Windows.UI.Xaml.Media.Imaging;
    using Windows.UI.Xaml.Shapes;
    using WindowsPreview.Kinect;
    using WindowsPreview.Kinect.Input;
    /// <summary>
    /// Página vacía que se puede usar de forma independiente o a la que se puede navegar dentro de un objeto Frame.
    /// </summary>
    public sealed partial class ColorBonesViewModel : ViewModel
    {

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Kinect DPI.
        /// </summary>
        public static readonly double DPI = 96.0;

        /// <summary>
        /// Intermediate storage for receiving frame data from the sensor
        /// </summary>
        private byte[] pixels = null;

        /// <summary>
        /// The bitmap source.
        /// </summary>
        static WriteableBitmap _bitmap = null;

        /// <summary>
        /// Font size of face property text 
        /// </summary>
        private const double DrawTextFontSize = 30;

        /// <summary>
        /// Number of bodies tracked
        /// </summary>
        private int bodyCount;

        private List<double> myList = new List<double>();

        private double angle;

        private double median;

        private int confra;

        private int pitch, yaw, roll;

        /// <summary>
        /// Reader for color frames
        /// </summary>
        private MultiSourceFrameReader reader;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private IList<Body> bodies = null;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;


        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;


        /// <summary>
        /// Initializes a new instance of the MainPage class.
        /// </summary>
        public ColorBonesViewModel()
        {
            this.Loaded += OnLoaded;
            this.Unloaded += OnUnloaded;

        }

        void OnLoaded(object sender, RoutedEventArgs args)
        {
            this.kinectSensor = KinectSensor.GetDefault();
            if (this.kinectSensor != null)
            {
                // open the reader for the color frames
                if (!this.kinectSensor.IsOpen) this.kinectSensor.Open();

                reader = this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Body);
            }
            if (reader != null)
            {
                reader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;
            }
        }

        // Data binding source for the device status TextBlock
        public string KinectStatus
        {
            get
            {
                if (kinectSensor == null) return "Off";
                return kinectSensor.IsAvailable ? "Available" : "Not available";
            }
        }

        private void OnPropertyChanged([CallerMemberName] string propertyName = null)
        {
            var handler = PropertyChanged;
            if (handler != null) handler(this, new PropertyChangedEventArgs(propertyName));
        }    

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void OnUnloaded(object sender, RoutedEventArgs e)
        {
             // Dispose of the frame reader and allow GC
            if(reader != null)
            {
             reader.Dispose();
              reader = null;
            }

            // Dispose of the Kinect device and allow GC
            if(kinectSensor == null) return;

            kinectSensor.Close();
            kinectSensor = null;
           
        }

        // This is the Source for the Image control
        public WriteableBitmap Bitmap
        {
            get { return _bitmap; }
            set { _bitmap = value; OnPropertyChanged(); }
        }

        /// <summary>
        /// Handles the color frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            // Color
            try
            {
                var frameReference = e.FrameReference.AcquireFrame();

                using (var colorFrame = frameReference.ColorFrameReference.AcquireFrame())
                {
                    if (colorFrame != null)
                    {

                        // Get info on what format frames will take as they arrive
                        var colorFrameDescription = kinectSensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Rgba);

                        // ColorFrame is IDisposable
                        displayWidth = colorFrameDescription.Width;
                        displayHeight = colorFrameDescription.Height;

                        pixels = new byte[this.displayWidth * this.displayHeight * colorFrameDescription.BytesPerPixel];

                        if (_bitmap == null)
                        {
                            // get size of joint space
                            _bitmap = new WriteableBitmap(displayWidth, displayHeight);
                        }

                        // Copy the image data into the temp pixel buffer (we have to use an intermdiate 
                        // byte array buffer as we can't  access the bitmap's buffer directly as an array of bytes)
                        if (colorFrame.RawColorImageFormat == ColorImageFormat.Bgra)
                            colorFrame.CopyRawFrameDataToArray(pixels);
                        else
                            colorFrame.CopyConvertedFrameDataToArray(pixels, ColorImageFormat.Bgra);

                        // Write the contents of the temp pixel buffer into the re-writable bitmap's buffer
                        _bitmap.PixelBuffer.AsStream().Write(pixels, 0, pixels.Length);
                        _bitmap.Invalidate(); // Invalidating the bitmap forces it to be redrawn
                        image.Source = _bitmap;
                    }
                }

                // Body
                using (var frame = frameReference.BodyFrameReference.AcquireFrame())
                {
                    if (frame != null)
                    {
                        // BodyFrame is IDisposable
                        canvas.Children.Clear();

                        bodies = new Body[frame.BodyFrameSource.BodyCount];

                        // set the maximum number of bodies that would be tracked by Kinect
                        this.bodyCount = kinectSensor.BodyFrameSource.BodyCount;

                        // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                        // As long as those body objects are not disposed and not set to null in the array,
                        // those body objects will be re-used.
                        frame.GetAndRefreshBodyData(bodies);
                        foreach (Body body in bodies)
                        {
                            if (body.IsTracked)
                            {
                                IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                                // convert the joint points to depth (display) space
                                Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                                foreach (Joint joint in body.Joints.Values)
                                {
                                    if (joint.TrackingState == TrackingState.Tracked)
                                    {
                                        // 3D space point
                                        CameraSpacePoint position = joint.Position;

                                        // 2D space point
                                        Point point = new Point();

                                        ColorSpacePoint colorPoint = kinectSensor.CoordinateMapper.MapCameraPointToColorSpace(position);

                                        point.X = ((float.IsInfinity(colorPoint.X) ? 0 : colorPoint.X) - 50) / 1.2;
                                        point.Y = ((float.IsInfinity(colorPoint.Y) ? 0 : colorPoint.Y) - 50) / 1.1;

                                        jointPoints[joint.JointType] = point;

                                        DrawJoint(canvas, point);
                                    }
                                }
                                this.DrawBody(joints, jointPoints, canvas);
                                angle = AngleBetweenJoints(joints[JointType.WristRight], joints[JointType.ElbowRight], joints[JointType.ShoulderRight]);                                
                                myList.Add(angle);
                                //confra++;
                                if (confra == 0)
                                {
                                    //CalculateMedian(myList, out median);
                                    RotationResults(JointType.WristRight, body, out pitch, out yaw, out roll);
                                    confra = 0;
                                }
                            }
                            //this.WriteAngle(median);
                            WriteAngle(pitch, yaw, roll);
                        }
                    }
                }
            }
            catch (Exception)
            {
            }
        }


        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, Canvas c)
        {
            // Draw the bones
            // Torso
            this.DrawBone(joints, jointPoints, JointType.Head, JointType.Neck, c);
            this.DrawBone(joints, jointPoints, JointType.Neck, JointType.SpineShoulder, c);
            this.DrawBone(joints, jointPoints, JointType.SpineShoulder, JointType.SpineMid, c);
            this.DrawBone(joints, jointPoints, JointType.SpineMid, JointType.SpineBase, c);
            this.DrawBone(joints, jointPoints, JointType.SpineShoulder, JointType.ShoulderRight, c);
            this.DrawBone(joints, jointPoints, JointType.SpineShoulder, JointType.ShoulderLeft, c);
            this.DrawBone(joints, jointPoints, JointType.SpineBase, JointType.HipRight, c);
            this.DrawBone(joints, jointPoints, JointType.SpineBase, JointType.HipLeft, c);

            // Right Arm    
            this.DrawBone(joints, jointPoints, JointType.ShoulderRight, JointType.ElbowRight, c);
            this.DrawBone(joints, jointPoints, JointType.ElbowRight, JointType.WristRight, c);
            this.DrawBone(joints, jointPoints, JointType.WristRight, JointType.HandRight, c);
            this.DrawBone(joints, jointPoints, JointType.HandRight, JointType.HandTipRight, c);
            this.DrawBone(joints, jointPoints, JointType.WristRight, JointType.ThumbRight, c);

            // Left Arm
            this.DrawBone(joints, jointPoints, JointType.ShoulderLeft, JointType.ElbowLeft, c);
            this.DrawBone(joints, jointPoints, JointType.ElbowLeft, JointType.WristLeft, c);
            this.DrawBone(joints, jointPoints, JointType.WristLeft, JointType.HandLeft, c);
            this.DrawBone(joints, jointPoints, JointType.HandLeft, JointType.HandTipLeft, c);
            this.DrawBone(joints, jointPoints, JointType.WristLeft, JointType.ThumbLeft, c);

            // Right Leg
            this.DrawBone(joints, jointPoints, JointType.HipRight, JointType.KneeRight, c);
            this.DrawBone(joints, jointPoints, JointType.KneeRight, JointType.AnkleRight, c);
            this.DrawBone(joints, jointPoints, JointType.AnkleRight, JointType.FootRight, c);

            // Left Leg
            this.DrawBone(joints, jointPoints, JointType.HipLeft, JointType.KneeLeft, c);
            this.DrawBone(joints, jointPoints, JointType.KneeLeft, JointType.AnkleLeft, c);
            this.DrawBone(joints, jointPoints, JointType.AnkleLeft, JointType.FootLeft, c);
        }

        public void DrawJoint(Canvas canvas, Point point)
        {
            // Create an ellipse.
            Ellipse ellipse = new Ellipse
            {
                Width = 20,
                Height = 20,
                Fill = new SolidColorBrush(Colors.LightBlue)
            };

            // Position the ellipse according to the point's coordinates.
            Canvas.SetLeft(ellipse, point.X - ellipse.Width / 2);
            Canvas.SetTop(ellipse, point.Y - ellipse.Height / 2);

            // Add the ellipse to the canvas.
            canvas.Children.Add(ellipse);
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, Canvas c)
        {
            Joint joint0 = joints[jointType0];

            Joint joint1 = joints[jointType1];
            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                Line line = new Line
                {
                    X1 = (float)jointPoints[jointType0].X,
                    Y1 = (float)jointPoints[jointType0].Y,
                    X2 = (float)jointPoints[jointType1].X,
                    Y2 = (float)jointPoints[jointType1].Y,
                    StrokeThickness = 5,
                    Stroke = new SolidColorBrush(Colors.LightBlue)
                };
                canvas.Children.Add(line);
            }
        }

        /// <summary>
        /// Regresa el ángulo interno dadas 3 Joints
        /// </summary>
        /// <param name="j1"></param>
        /// <param name="j2"></param>
        /// <param name="j3"></param>
        /// <returns></returns>
        public static double AngleBetweenJoints(Joint j1, Joint j2, Joint j3)
        {
            double Angulo = 0;
            double shrhX = j1.Position.X - j2.Position.X;
            double shrhY = j1.Position.Y - j2.Position.Y;
            double shrhZ = j1.Position.Z - j2.Position.Z;
            double hsl = vectorNorm(shrhX, shrhY, shrhZ);
            double unrhX = j3.Position.X - j2.Position.X;
            double unrhY = j3.Position.Y - j2.Position.Y;
            double unrhZ = j3.Position.Z - j2.Position.Z;
            double hul = vectorNorm(unrhX, unrhY, unrhZ);
            double mhshu = shrhX * unrhX + shrhY * unrhY + shrhZ * unrhZ;
            double x = mhshu / (hul * hsl);
            if (x != Double.NaN)
            {
                if (-1 <= x && x <= 1)
                {
                    double angleRad = Math.Acos(x);
                    Angulo = angleRad * (180.0 / Math.PI);
                }
                else
                    Angulo = 0;
            }
            else
                Angulo = 0;
            return Angulo;
        }

        /// <summary>
        /// Euclidean norm of 3-component Vector
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="z"></param>
        /// <returns></returns>
        private static double vectorNorm(double x, double y, double z)
        {
            return Math.Sqrt(Math.Pow(x, 2) + Math.Pow(y, 2) + Math.Pow(z, 2));
        }

        private void WriteAngle(double p)
        {

            this.anglTextbox.Text = string.Format(CultureInfo.CurrentCulture, p.ToString());
            this.anglTextbox.FontSize = 20;
        }

        private void WriteAngle(int pitch, int yaw, int roll)
        {

            this.anglTextbox.Text = "JointYaw : " + yaw + "\n" +
                        "JointPitch : " + pitch + "\n" +
                        "JointRoll : " + roll + "\n";
            this.anglTextbox.FontSize = 20;
        }

        /// <summary>
        /// Draws face frame results
        /// </summary>
        /// <param name="faceIndex">the index of the face frame corresponding to a specific body in the FOV</param>
        /// <param name="faceResult">container of all face frame results</param>
        /// <param name="drawingContext">drawing context to render to</param>
        private void RotationResults(JointType joint, Body bod, out int pitch, out int yaw, out int roll)
        {
            string jointText = string.Empty;
            Vector4 quaternion = new Vector4();
            IReadOnlyDictionary<JointType, JointOrientation> orientations = bod.JointOrientations;
            quaternion.X = bod.JointOrientations[joint].Orientation.X;
            quaternion.Y = bod.JointOrientations[joint].Orientation.Y;
            quaternion.Z = bod.JointOrientations[joint].Orientation.Z;
            quaternion.W = bod.JointOrientations[joint].Orientation.W;

            ExtractRotationInDegrees(quaternion, out pitch, out yaw, out roll);
        }

        /// <summary>
        /// Converts rotation quaternion to Euler angles 
        /// And then maps them to a specified range of values to control the refresh rate
        /// </summary>
        /// <param name="rotQuaternion">face rotation quaternion</param>
        /// <param name="pitch">rotation about the X-axis</param>
        /// <param name="yaw">rotation about the Y-axis</param>
        /// <param name="roll">rotation about the Z-axis</param>
        private static void ExtractRotationInDegrees(Vector4 rotQuaternion, out int pitch, out int yaw, out int roll)
        {
            double x = rotQuaternion.X;
            double y = rotQuaternion.Y;
            double z = rotQuaternion.Z;
            double w = rotQuaternion.W;

            // convierte la rotacion a angulos de Euler, en grados
            double yawD, pitchD, rollD;
            pitchD = Math.Atan2(2 * ((y * z) + (w * x)), (w * w) - (x * x) - (y * y) + (z * z)) / Math.PI * 180.0;
            yawD = Math.Asin(2 * ((w * y) - (x * z))) / Math.PI * 180.0;
            rollD = Math.Atan2(2 * ((x * y) + (w * z)), (w * w) + (x * x) - (y * y) - (z * z)) / Math.PI * 180.0;

            // limita los valores a un múltiplo de un incremento específico que controle el ratio de actualizaciones
            double increment = 1.0;
            pitch = (int)(Math.Floor((pitchD + ((increment / 2.0) * (pitchD > 0 ? 1.0 : -1.0))) / increment) * increment);
            yaw = (int)(Math.Floor((yawD + ((increment / 2.0) * (yawD > 0 ? 1.0 : -1.0))) / increment) * increment);
            roll = (int)(Math.Floor((rollD + ((increment / 2.0) * (rollD > 0 ? 1.0 : -1.0))) / increment) * increment);
        }

        /// <summary>
        /// Converts rotation quaternion to Euler angles 
        /// And then maps them to a specified range of values to control the refresh rate
        /// </summary>
        /// <param name="rotQuaternion">face rotation quaternion</param>
        /// <param name="pitch">rotation about the X-axis</param>
        /// <param name="yaw">rotation about the Y-axis</param>
        /// <param name="roll">rotation about the Z-axis</param>
        private void CalculateMedian(List<double> myList, out double median)
        {
            var query = from numbers in myList //select the numbers
                        orderby numbers ascending
                        select numbers;

            if (myList.Count % 2 == 0)
            { //we know its even
                int element = myList.Count / 2; ;
                 median = (double)((query.ElementAt(element - 1) + query.ElementAt(element)) / 2);
            }
            else
            {
                //we know its odd
                double element = (double)myList.Count / 2;
                element = Math.Round(element, MidpointRounding.AwayFromZero);
                median = (double)query.ElementAt((int)(element - 1));
            }
        }
    }

}
