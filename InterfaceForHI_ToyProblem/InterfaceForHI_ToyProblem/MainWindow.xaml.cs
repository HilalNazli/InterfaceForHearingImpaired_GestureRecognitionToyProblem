using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Microsoft.Kinect;
using Accord.Controls;
using Accord.IO;
using Accord.Statistics.Models.Markov;
using Accord.Statistics.Models.Markov.Learning;
using Accord.Math;
using Accord.Statistics.Analysis;
using Accord.Statistics.Models.Markov.Topology;
using Accord.Statistics.Distributions.Fitting;
using Accord.Statistics.Distributions.Multivariate;
using Accord.Statistics.Models.Fields;
using Accord.Statistics.Models.Fields.Functions;
using Accord.Statistics.Models.Fields.Learning;
using Accord.Statistics.Models.Markov;
namespace InterfaceForHI_ToyProblem
{

    public class Borders
    {
        public string label { get; set; }
        public int start { get; set; }
        public int end { get; set; }
    }

    public class Coordinate
    {
        public double rx { get; set; }
        public double ry { get; set; }
        public double rz { get; set; }
        public double lx { get; set; }
        public double ly { get; set; }
        public double lz { get; set; }
    }

    public partial class MainWindow : Window
    {
        #region Variables
        /////////////////////////////////////////////////////////
        /// Active Kinect sensor
        private KinectSensor kinectSensor = null;

        /// Size of the RGB pixel in the bitmap
        private readonly int bytesPerPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;


        // MultiSourceFrame Variables
        MultiSourceFrameReader displayMultiSourceFrameReader = null;
        FrameSourceTypes myFrameSources = FrameSourceTypes.Color;

        MultiSourceFrameReader recordMultiSourceFrameReader = null;

        StringBuilder myCSVwriter = new StringBuilder();
        string folderPath = "";

        int nFrames = 0;

        /// Current status text to display
        private string statusText = null;
       
        // Color Image Variables
        private WriteableBitmap colorImageBitmap = null;
        private byte[] colorImage = null;

        private CoordinateMapper coordinateMapper = null;
        private Body[] bodies = null;

        // Body Index
        private byte[] displayPixels = null;
        private ColorSpacePoint[] colorPoints = null;
        private WriteableBitmap bodyIndexImageBitmap = null;
        private byte[] bodyIndexFrameData = null;


        // Record Flags
        private Boolean recording = false;


        Person person = new Person();
        double ratio = 0;
        Boolean isRepeated = false;
        Boolean isInQuestion4 = false;
        int count = 0;
        int letItLoadCount = 0;
        int videoTrickCount = 0;
        double svAnswersHorizontalOffset = 0;

        System.Windows.Threading.DispatcherTimer dispatcherTimerForCamera = new System.Windows.Threading.DispatcherTimer();
        #endregion

        #region HMM
        HiddenMarkovClassifier<MultivariateNormalDistribution> hmmc;
        double[][] bodyFeaturesSequence;
        double [][][] tr_data;
        double [][][] te_data;
        // Labels for the sequences
        string[] tr_labels; //either "sick" or "info" };
        string[] te_labels;
        int[] tr_labelsInt;
        int[] te_labelsInt;
        // States for the sequences
        int[] states;
        int totalNumOfInstances;
        #endregion
        public MainWindow()
        {  
            bodyFeaturesSequence = new double[350][];

            //trainingData = readTrainingData();


            string[] searchedLabels = new string[2];
            searchedLabels[0] = "ben_hastayim";
            searchedLabels[1] = "bilgi_almak_istiyorum_ben";
            string[] searchedLabels_te = new string[2];
            searchedLabels_te[0] = "ben_hastayim_te";
            searchedLabels_te[1] = "bilgi_almak_istiyorum_ben_te";

            string DataPath = "C:\\Users\\HilalNazli\\Desktop\\Cmpe492\\InterfaceForHI_ToyProblem\\InterfaceForHI_ToyProblem\\data";
            string subPath = "C:\\Users\\HilalNazli\\Desktop\\Cmpe492\\InterfaceForHI_ToyProblem\\InterfaceForHI_ToyProblem\\data\\..\\parsed";
            //int[] actionSizes1 = parseDataToFiles(searchedLabels[0], DataPath, subPath);
            //int[] actionSizes2 = parseDataToFiles(searchedLabels[1], DataPath, subPath);
            readDataToArray_tr(subPath, 20, 20,searchedLabels);
            readDataToArray_te(subPath, 12, 15, searchedLabels_te);

            //Console.WriteLine(tr_data.Length);
            //Initialize hmm
            // Get the number of different classes in the data
            totalNumOfInstances = tr_data.Length;


            // Creates a new hidden Markov classifier for the number of classes
            //hmmc = new HiddenMarkovClassifier(2, states, 2, labels);
            
            hmmc = new HiddenMarkovClassifier<MultivariateNormalDistribution>(2,
                new Forward(6), new MultivariateNormalDistribution(2), searchedLabels);

            //save var!!!!
            //Train hmm
            trainHMMC();

            //Test hmm
            //
            int misclassifications = 0;
            for (int i = 0; i < 12; i++) {
               double[][] testInstance = te_data[i];
                int assignedClass = hmmc.Compute(testInstance);
                 if (assignedClass == 1)
                {
                    misclassifications++;
                }
            }
            for (int i = 0; i < 15; i++)
            {
                double[][] testInstance = te_data[12+i];
                int assignedClass = hmmc.Compute(testInstance);
                if (assignedClass == 0)
                {
                    misclassifications++;
                }
            }
            Console.WriteLine("Number of misclassified test instances: "+misclassifications);
                /////////////////////////////////////////////////////////
                // for Alpha, one sensor is supported
                this.kinectSensor = KinectSensor.GetDefault();

            if (this.kinectSensor != null)
            {

                // open the sensor
                this.kinectSensor.Open();

                // Display Multi Source Frame Reader
                this.displayMultiSourceFrameReader = this.kinectSensor.OpenMultiSourceFrameReader(myFrameSources);

                // Color Variable Initialization
                FrameDescription colorFrameDescription = this.kinectSensor.ColorFrameSource.FrameDescription;
                this.colorImage = new byte[colorFrameDescription.Width * colorFrameDescription.Height * this.bytesPerPixel];
                this.colorImageBitmap = new WriteableBitmap(colorFrameDescription.Width, colorFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);
                
                // Body Variable Initialization
                this.coordinateMapper = this.kinectSensor.CoordinateMapper;
                this.bodies = new Body[6]; //[this.kinectSensor.BodyFrameSource.BodyCount];

            }
            /////////////////////////////////////////////////////////

            InitializeComponent();
            mainWindow.WindowState = WindowState.Maximized;
            ratio = System.Windows.SystemParameters.PrimaryScreenHeight / mainWindow.Height;

            mainWindow.Width = System.Windows.SystemParameters.PrimaryScreenWidth;
            mainWindow.Height = System.Windows.SystemParameters.PrimaryScreenHeight;



            //1- Question is loaded and played once(or twice)
            //2- Answers are shown. Camera is on, for 5 seconds.
            //3- Question is shown again (for now). 

            loadQuestion1();
            initializeSizes(ratio);

            this.imgDisplayImage.Source = colorImageSource;
        }

        /// <summary>
        ///   Trains the hidden Markov classifier
        /// </summary>
        /// 
        private void trainHMMC()
        {
            
            if (hmmc == null)
            {
                MessageBox.Show("Please create a sequence classifier first.");
                return;
            }
       

            // Grab training parameters
            int iterations = 0;
            double limit = 0.001;
            bool rejection = false;

            // Create a new hidden Markov model learning algorithm
            var teacher = new HiddenMarkovClassifierLearning<MultivariateNormalDistribution>(hmmc, i =>
            {
                return new BaumWelchLearning<MultivariateNormalDistribution>(hmmc.Models[i])
                {
                    Iterations = iterations,
                    Tolerance = limit,
                    FittingOptions = new NormalOptions()
                    {
                        Regularization = 1e-5
                    }
                };
            });
            teacher.Empirical = true;
            teacher.Rejection = rejection;

            // Learn the classifier
            double error = teacher.Run(tr_data, tr_labelsInt);

           
          
        }

   


        public ImageSource colorImageSource
        {
            get
            {
                return this.colorImageBitmap;
            }
        }



        /// Execute start up tasks
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.displayMultiSourceFrameReader != null)
            {
                //Eskidendi.
                // this.displayMultiSourceFrameReader.MultiSourceFrameArrived += this.Reader_DisplayMultiSourceFrameArrived;
            }
        }
        #region Initializations
        private void initializeSizes(double ratio)
        {

            //Original sizes of all objects
            double heightOfIDropdown_ico = 64;
            double widthOfIDropdown_ico = 64;

            double heightOfIUser_ico = 64;
            double widthOfIUser_ico = 64;

            double heightOfIRe = 128;
            double widthOfIRe = 128;

            double fontSizeOfTBQuestion = 48;

            double heightOfI_Prev = 240;
            double widthOfI_Prev = 64;

            //Add additional things in slide bar here
            //double heightOfspAnswers = 350;
            //double widthOfspAnswers = 960;
            //double widthOfsvAnswers = 960;
            double heightOfsvAnswers = 350;

            double heightOfI_Next = 240;
            double widthOfI_Next = 64;

            double heightOfIBoun_ = 85;
            double widthOfIBoun_ = 85;

            double heightOfIPi_ = 85;
            double widthOfIPi_ = 85;

            double fontSizeOfTBProject = 24;

            double heightOfTBName = 48;
            double widthOfTbName = 480;
            double fontOfTBName = 24;

            double heightOfBName = 48;
            double widthOfBName = 100;
            double fontOfBName = 24;

            double heightOfMeMainVideo = 480;

            double widthOfLVMenu = 280;
            //Resize everything with THE ratio yeah!
            mainWindow.IDropdown_ico.Height = heightOfIDropdown_ico * ratio;
            mainWindow.IDropdown_ico.Width = widthOfIDropdown_ico * ratio;

            mainWindow.IUser_ico.Height = heightOfIUser_ico * ratio;
            mainWindow.IUser_ico.Width = widthOfIUser_ico * ratio;

            mainWindow.IRe.Height = heightOfIRe * ratio;
            mainWindow.IRe.Width = widthOfIRe * ratio;

            mainWindow.TBQuestion.FontSize = fontSizeOfTBQuestion * ratio;

            mainWindow.I_Prev.Height = heightOfI_Prev * ratio;
            mainWindow.I_Prev.Width = widthOfI_Prev * ratio;


            //Add additional things in slide bar here
            //mainWindow.spAnswers.Height = heightOfspAnswers * ratio;
            // mainWindow.spAnswers.Width = widthOfspAnswers * ratio;
            //mainWindow.svAnswers.Width = widthOfsvAnswers * ratio;
            mainWindow.svAnswers.Height = heightOfsvAnswers * ratio;

            mainWindow.I_Next.Height = heightOfI_Next * ratio;
            mainWindow.I_Next.Width = widthOfI_Next * ratio;

            mainWindow.IBoun_.Height = heightOfIBoun_ * ratio;
            mainWindow.IBoun_.Width = widthOfIBoun_ * ratio;

            mainWindow.IPi_.Height = heightOfIPi_ * ratio;
            mainWindow.IPi_.Width = widthOfIPi_ * ratio;

            mainWindow.TBProject.FontSize = fontSizeOfTBProject * ratio;

            // mainWindow.meMainVideo.Height = mainWindow.Row1.ActualHeight; 
            mainWindow.meMainVideo.Height = heightOfMeMainVideo * ratio;
            mainWindow.imgDisplayImage.Height = heightOfMeMainVideo * ratio;
            //Resize margins

            tbName.Height = heightOfTBName * ratio;
            tbName.Width = widthOfTbName * ratio;
            tbName.FontSize = fontOfTBName * ratio;

            bName.Height = heightOfBName * ratio;
            bName.Width = widthOfBName * ratio;
            bName.FontSize = fontOfBName * ratio;
            bOtherName.Height = heightOfBName * ratio;
            bOtherName.Width = widthOfBName * ratio;
            bOtherName.FontSize = fontOfBName * ratio;

            lvMenu.Width = widthOfLVMenu * ratio;

        }


        private void initializeAnswerVideoProperties(object sender, RoutedEventArgs args)
        {
            MediaElement me = (MediaElement)args.Source;
            DockPanel.SetDock(me, Dock.Top);
            me.Height = 240 * ratio;

            me.LoadedBehavior = System.Windows.Controls.MediaState.Manual;
            me.MediaEnded += new RoutedEventHandler(AnswerMediaEnded);
            me.MouseLeftButtonDown += new MouseButtonEventHandler(AnswerMediaMouseLeftButtonDown);
            //me.Stretch = Stretch.Uniform;

        }
        private void initializeAnswerTextProperties(object sender, RoutedEventArgs args)
        {
            TextBlock tb = (TextBlock)args.Source;
            tb.FontSize = 24 * ratio;
            //tb.Width = 100 * ratio;
            tb.TextAlignment = TextAlignment.Center;
            tb.TextWrapping = TextWrapping.Wrap;
            tb.HorizontalAlignment = System.Windows.HorizontalAlignment.Center;
            tb.Foreground = (System.Windows.Media.Brush)mainWindow.Resources["BlackPen"];
        }

        private void initializeAnswerGridProperties(object sender, RoutedEventArgs args)
        {
            Grid g = (Grid)args.Source;
            Thickness margin = new System.Windows.Thickness();
            margin.Left = 32 * ratio;
            g.Margin = margin;
            g.Width = 295 * ratio;
        }
        private void initializeAnswerDockPanelProperties(object sender, RoutedEventArgs args)
        {
            DockPanel dp = (DockPanel)args.Source;
            dp.HorizontalAlignment = System.Windows.HorizontalAlignment.Center;

        }
        #endregion
        #region Events

        private void AnswerMediaEnded(object sender, RoutedEventArgs args)
        {
            MediaElement me = (MediaElement)args.Source;
            me.Position = System.TimeSpan.Zero;
            me.Play();
        }

        private void AnswerMediaMouseLeftButtonDown(object sender, MouseButtonEventArgs args)
        {
           //Taninan isarete gore hareket edecegiz.

        }
        private void listViewItemMouseDown(object sender, MouseButtonEventArgs args)
        {
            //Burada kullanilmayacak.

        }

        private void meMainVideo_MediaEnded(object sender, RoutedEventArgs e)
        {


            if (!isRepeated)
            {
                mainWindow.svAnswers.Visibility = System.Windows.Visibility.Visible;
                playAllChildren(spAnswers);
               
            }

            //restartVideo();

            isRepeated = true;

            //
            //Do not restart.
            //When the media is ended, make it hidden, so that the user can see himself/herself on the screen.
            //Set timer to 5 seconds. Record the user for 5 seconds and then restrart the video.
            mainWindow.meMainVideo.Visibility = System.Windows.Visibility.Hidden;
            mainWindow.imgDisplayImage.Visibility = System.Windows.Visibility.Visible;
            mainWindow.MiddleLineBorder.Background = (System.Windows.Media.Brush)mainWindow.Resources["GreenBrush"];
            mainWindow.MiddleLineBorder_.Background = (System.Windows.Media.Brush)mainWindow.Resources["GreenBrush_"];


            //System.Console.WriteLine("nFram1e:" + nFrames);

            //StartRecording
            recording = true;
            
            nFrames = 0;
           
            //null control
            FrameSourceTypes recordFrameSources = FrameSourceTypes.Body | FrameSourceTypes.Color;

            Thread.Sleep(100);
            if (recordMultiSourceFrameReader == null)
           {
               // Open the recorder MultiSourceFrameReader.
               this.recordMultiSourceFrameReader = this.kinectSensor.OpenMultiSourceFrameReader(recordFrameSources);
               this.recordMultiSourceFrameReader.MultiSourceFrameArrived += this.Reader_RecordMultiSourceFrameArrived;
               //this.recordMultiSourceFrameReader.MultiSourceFrameArrived += this.Reader_DisplayMultiSourceFrameArrived;
 
 
               dispatcherTimerForCamera.Tick += new EventHandler(dispatcherTimer_forCamera_Tick);
               dispatcherTimerForCamera.Interval = new TimeSpan(0, 0, 5);
               dispatcherTimerForCamera.Start();

           }
           else {
               System.Console.WriteLine("UPS!!!");
           }
        }

      
        private void dispatcherTimer_forCamera_Tick(object sender, EventArgs e)
        {
            dispatcherTimerForCamera.Stop();

            //StopRecording
            // Here, blurr the screen and status: processing your answer

            //Restart the question video for now.
            //Later we will process what we've recorded.
            recording = false;


            // Dispose the Recording MultiSourceFrameReader.
            if (recordMultiSourceFrameReader != null)
            {
                this.recordMultiSourceFrameReader.Dispose();
                this.recordMultiSourceFrameReader = null;
            }
            Thread.Sleep(100);
            Console.WriteLine("Number of Frames " + nFrames);

            //Work with bodyFramesSequence and hmmc here
            //sample.RecognizedAs = hmm.Compute(sample.Input);

            double[][] currentTestInstance = new double[nFrames][];
            currentTestInstance = bodyFeaturesSequence.Take(nFrames).ToArray();

            //Apply normalization
            double startingX = currentTestInstance[0][0];
            double startingY = currentTestInstance[0][1];
            double maxX = -999;
            double maxY = -999; 
            for (int i = 0; i < nFrames; i++) {
                currentTestInstance[i][0] = currentTestInstance[i][0] - startingX;
                currentTestInstance[i][1] = currentTestInstance[i][1] - startingY;
                if (Math.Abs(currentTestInstance[i][0]) > maxX) {
                    maxX = Math.Abs(currentTestInstance[i][0]);
                }
                if (Math.Abs(currentTestInstance[i][1]) > maxY)
                {
                    maxY = Math.Abs(currentTestInstance[i][1]);
                }
            }
            for (int i = 0; i < nFrames; i++){
                currentTestInstance[i][0] = currentTestInstance[i][0] / maxX;
                currentTestInstance[i][1] = currentTestInstance[i][1] / maxY;
                Console.WriteLine(currentTestInstance[i][0] + "  " + currentTestInstance[i][1]);

            }
          
            int assignedClass = hmmc.Compute(currentTestInstance);
            //Show your decision in huge letters!!!
            tbGuessedAnswer.Visibility = System.Windows.Visibility.Visible;
            mainWindow.imgDisplayImage.Visibility = System.Windows.Visibility.Hidden;
            if (assignedClass == 0)
            {
                Console.WriteLine("Aldigimiz Sonuc: HASTAYMIS");
                tbGuessedAnswer.Text = "Hastayım.";
            }
            else if(assignedClass==1){
                Console.WriteLine("Aldigimiz Sonuc: BILGI ISTIYORMUS");
                tbGuessedAnswer.Text = "Bilgi almak istiyorum.";


            }
            
            /*
            mainWindow.meMainVideo.Visibility = System.Windows.Visibility.Visible;
            mainWindow.imgDisplayImage.Visibility = System.Windows.Visibility.Hidden;

            mainWindow.MiddleLineBorder.Background = (System.Windows.Media.Brush)mainWindow.Resources["OrangeBrush"];
            mainWindow.MiddleLineBorder_.Background = (System.Windows.Media.Brush)mainWindow.Resources["OrangeBrush_"];
            restartVideo();
            */
        }


        private void svAnswers_IsVisibleChanged(object sender, DependencyPropertyChangedEventArgs e)
        {
            //Burada kullanilmayacak.
        }

      
        private void bOtherName_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            person.otherPatientName = tbName.Text;
            person.programFlow = person.programFlow + "Ziyaret etmek istediği hastanın ismi: " + tbName.Text + "\n";
            System.Diagnostics.Debug.WriteLine(person.programFlow);
            ListViewItem lvi = new ListViewItem();
            lvMenu.Items.Add(lvi);
            lvi.PreviewMouseDown += new MouseButtonEventHandler(listViewItemMouseDown);
            lvi.Content = "Hasta İsmi";

            //loadDirection2();
            bOtherName.Visibility = System.Windows.Visibility.Hidden;
            spInput.Visibility = System.Windows.Visibility.Hidden;


        }

        private void bName_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {

            person.name = tbName.Text;
            person.programFlow = person.programFlow + "Hastanın ismi: " + tbName.Text + "\n";
            System.Diagnostics.Debug.WriteLine(person.programFlow);
            ListViewItem lvi = new ListViewItem();
            lvMenu.Items.Add(lvi);
            lvi.PreviewMouseDown += new MouseButtonEventHandler(listViewItemMouseDown);
            lvi.Content = "İsim";

            //loadDirection2();
            bName.Visibility = System.Windows.Visibility.Hidden;
            spInput.Visibility = System.Windows.Visibility.Hidden;

        }
        //This will not be used here.
        private void I_Next_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            svAnswers.LineRight();
            svAnswers.LineRight();
            svAnswers.LineRight();
        }
        //This will not be used here.
        private void I_Prev_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {

            svAnswers.LineLeft();
            //decrementVideoTrickCount();
            svAnswers.LineLeft();
            //decrementVideoTrickCount();
            svAnswers.LineLeft();
           // decrementVideoTrickCount();
            
        }

        private void bClose_PreviewMouseDown(object sender, MouseButtonEventArgs e)
        {
            person.printOut();
            Application.Current.Shutdown();
        }

        private void I_Prev_IsVisibleChanged(object sender, DependencyPropertyChangedEventArgs e)
        {
            if (I_Prev.Visibility.Equals(System.Windows.Visibility.Visible))
            {
                spInput.Visibility = System.Windows.Visibility.Hidden;
                bClose.Visibility = System.Windows.Visibility.Hidden;
            }
        }

        private void IDropdown_ico_PreviewMouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            if (lvMenu.Visibility.Equals(System.Windows.Visibility.Visible))
            {
                lvMenu.Visibility = System.Windows.Visibility.Hidden;
            }
            else
            {
                lvMenu.Visibility = System.Windows.Visibility.Visible;
            }
        }
        #endregion 
        #region Functions
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        public event PropertyChangedEventHandler PropertyChanged;

        /// Gets or sets the current status text to display
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }
        public void playAllChildren(StackPanel stackPanel)
        {
            //System.Collections.Generic.List<MediaElement> meAnswers = new System.Collections.Generic.List<MediaElement>();
            foreach (Grid g in stackPanel.Children)
            {
                //System.Diagnostics.Debug.WriteLine("Grid found");
                DockPanel d = g.Children.OfType<DockPanel>().FirstOrDefault();
                MediaElement me = d.Children.OfType<MediaElement>().FirstOrDefault();
                me.Play();
            }
        }

        private void restartVideo()
        {
            meMainVideo.Position = System.TimeSpan.Zero;
            meMainVideo.Play();
        }
#endregion
        #region Question
        private void loadQuestion(String question, String questionPath, int answersCount, String[] answers, String[] answersPath)
        {
            //If clicked when camera is on.
            dispatcherTimerForCamera.Stop();

            mainWindow.meMainVideo.Visibility = System.Windows.Visibility.Visible;
            mainWindow.imgDisplayImage.Visibility = System.Windows.Visibility.Hidden;

            mainWindow.MiddleLineBorder.Background = (System.Windows.Media.Brush)mainWindow.Resources["OrangeBrush"];
            mainWindow.MiddleLineBorder_.Background = (System.Windows.Media.Brush)mainWindow.Resources["OrangeBrush_"];

            //Set the question in the main video
            TBQuestion.Text = question;
            meMainVideo.Source = new System.Uri(questionPath, UriKind.Relative);
            isRepeated = false;
            svAnswers.Visibility = System.Windows.Visibility.Hidden;
            restartVideo();
            //Clear the stack panel for the answers
            spAnswers.Children.Clear();
            //Fill the stack panel with answers
            for (int i = 0; i < answersCount; i++)
            {
                //ANSWER i
                //  Declare answer grid
                Grid g = new Grid();
                spAnswers.Children.Add(g);
                g.Loaded += new RoutedEventHandler(initializeAnswerGridProperties);
                //    Declare answer dock panel
                DockPanel dp = new DockPanel();
                g.Children.Add(dp);
                dp.Loaded += new RoutedEventHandler(initializeAnswerDockPanelProperties);
                //      Declare answer media element
                MediaElement m = new MediaElement();
                dp.Children.Add(m);
                m.Source = new System.Uri(answersPath[i], UriKind.Relative);
                m.Loaded += new RoutedEventHandler(initializeAnswerVideoProperties);
                //      Declare answer text block
                TextBlock tb = new TextBlock();
                dp.Children.Add(tb);
                tb.Text = answers[i];
                tb.Loaded += new RoutedEventHandler(initializeAnswerTextProperties);
            }
        }

        private void loadQuestion1()
        {
            svAnswers.Visibility = System.Windows.Visibility.Hidden;
            ListViewItem lvi = new ListViewItem();
            lvi.PreviewMouseDown += new MouseButtonEventHandler(listViewItemMouseDown);
            String programFlow = person.programFlow;
            person = new Person();
            person.programFlow = programFlow;
            lvi.Content = "Giriş";
            lvMenu.Items.Clear();
            lvMenu.Items.Add(lvi);
            I_Next.Visibility = System.Windows.Visibility.Visible;
            I_Prev.Visibility = System.Windows.Visibility.Visible;
            String question = "Nasıl Yardımcı Olabilirim?";
            String questionPath = "SignVideos/Questions/Question1/NasilYardimciOlabilirim.mp4";
            int answersCount = 2;
            String[] answers = new String[] { 
                "Hastayım", 
                "Bilgi Almak İstiyorum" };
            String[] answersPath = new String[] { 
                "SignVideos/Answers/AnswersToQuestion1/BenHastayim.mp4",
                "SignVideos/Answers/AnswersToQuestion1/BilgiAlmakIstiyorum.mp4" };
            loadQuestion(question, questionPath, answersCount, answers, answersPath);
        }
        #endregion

    
        /// Execute shutdown tasks
        private void mainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.displayMultiSourceFrameReader != null)
            {
                this.displayMultiSourceFrameReader.Dispose();
                this.displayMultiSourceFrameReader = null;
            }

            if (this.recordMultiSourceFrameReader != null)
            {
                this.recordMultiSourceFrameReader.Dispose();
                this.recordMultiSourceFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }


        private void readDataToArray_tr(string subPath, int numberOfActions1, int numberOfActions2, string[] searchedLabels)
        {
         
            int totalNumberOfActions = numberOfActions1 + numberOfActions2;

            tr_data = new double[totalNumberOfActions][][];
            tr_labels = new string[totalNumberOfActions];
            states = new int[totalNumberOfActions];
            tr_labelsInt = new int[totalNumberOfActions];
            for (int i = 1; i <= numberOfActions1; i++)
            {
                string fileName = subPath + "\\" + searchedLabels[0] + "\\" + i + ".csv";
                //Console.WriteLine(fileName + " size of Action:" + actionSizes[i - 1]);

                TextReader tr = new StreamReader(fileName);

                int count = 0;
                for (int lineCount = 0; tr.Peek() != -1; lineCount++)
                {
                    string c = tr.ReadLine();
                    count++;
                }
                tr.Close();
                tr = new StreamReader(fileName);
                double[][] instances = new double[count][];

                for (int lineCount = 0; lineCount < count; lineCount++)
                {
                    string[] c = tr.ReadLine().Split(';');
                    double[] temp = new double[2];
                    temp[0] = Double.Parse(c[0]);
                    temp[1] = Double.Parse(c[1]);
                    instances[lineCount] = temp;
                }
                tr.Close();

                tr_data[i - 1] = instances; // All actions that are belong to the selected Label, from all data files.
                tr_labels[i - 1] = "sick";
                tr_labelsInt[i - 1] = 0;
            }

            for (int i = numberOfActions1 + 1; i <= totalNumberOfActions; i++)
            {
                string fileName = subPath + "\\" + searchedLabels[1] + "\\" + (i - numberOfActions1) + ".csv";
                //Console.WriteLine(fileName + " size of Action:" + actionSizes[i - 1]);

                TextReader tr = new StreamReader(fileName);
                int count = 0;
                for (int lineCount = 0; tr.Peek() != -1 ; lineCount++)
                {
                    String c = tr.ReadLine();
                    count++;
                }
                tr.Close();

                tr = new StreamReader(fileName);
                double[][] instances = new double[count][];

                for (int lineCount = 0; lineCount < count; lineCount++)
                {
                    String tmp = tr.ReadLine();

                    string[] c = tmp.Split(';');
                    double[] temp = new double[2];
                    temp[0] = Double.Parse(c[0]);
                    temp[1] = Double.Parse(c[1]);
                    instances[lineCount] = temp;

                }
                tr.Close();

                tr_data[i - 1] = instances; // All actions that are belong to the selected Label, from all data files.
                tr_labels[i - 1] = "info";
                tr_labelsInt[i - 1] = 1;
            }



        }
        private void readDataToArray_te(string subPath, int numberOfActions1, int numberOfActions2, string[] searchedLabels)
        {
         
            int totalNumberOfActions = numberOfActions1 + numberOfActions2;

            te_data = new double[totalNumberOfActions][][];
            te_labels = new string[totalNumberOfActions];
            te_labelsInt = new int[totalNumberOfActions];
            for (int i = 1; i <= numberOfActions1; i++)
            {
                string fileName = subPath + "\\" + searchedLabels[0] + "\\" + i + ".csv";
                //Console.WriteLine(fileName + " size of Action:" + actionSizes[i - 1]);

                TextReader tr = new StreamReader(fileName);

                int count = 0;
                for (int lineCount = 0; tr.Peek() != -1; lineCount++)
                {
                    string c = tr.ReadLine();
                    count++;
                }
                tr.Close();
                tr = new StreamReader(fileName);
                double[][] instances = new double[count][];

                for (int lineCount = 0; lineCount < count; lineCount++)
                {
                    string[] c = tr.ReadLine().Split(';');
                    double[] temp = new double[2];
                    temp[0] = Double.Parse(c[0]);
                    temp[1] = Double.Parse(c[1]);
                    instances[lineCount] = temp;
                }
                tr.Close();

                te_data[i - 1] = instances; // All actions that are belong to the selected Label, from all data files.
                te_labels[i - 1] = "sick";
                te_labelsInt[i - 1] = 0;
            }

            for (int i = numberOfActions1 + 1; i <= totalNumberOfActions; i++)
            {
                string fileName = subPath + "\\" + searchedLabels[1] + "\\" + (i - numberOfActions1) + ".csv";
                //Console.WriteLine(fileName + " size of Action:" + actionSizes[i - 1]);

                TextReader tr = new StreamReader(fileName);
                int count = 0;
                for (int lineCount = 0; tr.Peek() != -1 ; lineCount++)
                {
                    String c = tr.ReadLine();
                    count++;
                }
                tr.Close();

                tr = new StreamReader(fileName);
                double[][] instances = new double[count][];

                for (int lineCount = 0; lineCount < count; lineCount++)
                {
                    String tmp = tr.ReadLine();

                    string[] c = tmp.Split(';');
                    double[] temp = new double[2];
                    temp[0] = Double.Parse(c[0]);
                    temp[1] = Double.Parse(c[1]);
                    instances[lineCount] = temp;

                }
                tr.Close();

                te_data[i - 1] = instances; // All actions that are belong to the selected Label, from all data files.
                te_labels[i - 1] = "info";
                te_labelsInt[i - 1] = 1;
            }



        }
        private void readDataToArray(string subPath, int[] actionSizes1, int[] actionSizes2, string[] searchedLabels)
        {
            int numberOfActions1 = actionSizes1.Length;
            int numberOfActions2 = actionSizes2.Length;
            int totalNumberOfActions = numberOfActions1+numberOfActions2;

            tr_data = new double[totalNumberOfActions][][];
            tr_labels = new string[totalNumberOfActions];
            states = new int[totalNumberOfActions];
            tr_labelsInt = new int[totalNumberOfActions];
            for (int i = 1; i <= numberOfActions1; i++)
            {
                string fileName = subPath + "\\" + searchedLabels[0] + "\\" + i + ".csv";
                //Console.WriteLine(fileName + " size of Action:" + actionSizes[i - 1]);

                TextReader tr = new StreamReader(fileName);
                double[][] instances = new double[actionSizes1[i - 1]][];

                for (int lineCount = 0; lineCount < actionSizes1[i - 1]; lineCount++)
                {
                    string[] c = tr.ReadLine().Split(';');
                    double[] temp = new double[6];
                    temp[0] = Double.Parse(c[0]);
                    temp[1] = Double.Parse(c[1]);
                    temp[2] = Double.Parse(c[2]);
                    temp[3] = Double.Parse(c[3]);
                    temp[4] = Double.Parse(c[4]);
                    temp[5] = Double.Parse(c[5]);
                    instances[lineCount] = temp;
                }
                tr.Close();

                tr_data[i - 1] = instances; // All actions that are belong to the selected Label, from all data files.
                tr_labels[i - 1] = "sick";
                tr_labelsInt[i - 1] = 0;
            }

            for (int i = numberOfActions1+1; i <= totalNumberOfActions; i++)
            {
                string fileName = subPath + "\\" + searchedLabels[1] + "\\" + (i - numberOfActions1) + ".csv";
                //Console.WriteLine(fileName + " size of Action:" + actionSizes[i - 1]);

                TextReader tr = new StreamReader(fileName);
                double[][] instances = new double[actionSizes2[i-numberOfActions1- 1]][];

                for (int lineCount = 0; lineCount < actionSizes2[i-numberOfActions1 - 1]; lineCount++)
                {
                    String tmp = tr.ReadLine();
                 
                    string[] c = tmp.Split(';');
                    double[] temp = new double[6];
                    temp[0] = Double.Parse(c[0]);
                    temp[1] = Double.Parse(c[1]);
                    temp[2] = Double.Parse(c[2]);
                    temp[3] = Double.Parse(c[3]);
                    temp[4] = Double.Parse(c[4]);
                    temp[5] = Double.Parse(c[5]);
                    instances[lineCount] = temp;
                   
                }
                tr.Close();

                tr_data[i - 1] = instances; // All actions that are belong to the selected Label, from all data files.
                tr_labels[i - 1] = "info";
                tr_labelsInt[i - 1] = 1;
            }



        }

        private int[] parseDataToFiles(string searchedLabel, string DataPath, string subPath)
        {
            ArrayList list = new ArrayList();
            ArrayList borderlist = new ArrayList();
            ArrayList coordinateList = new ArrayList();
            subPath = subPath + "\\" + searchedLabel;

            list = GetSubDirectories(@DataPath);

            int totalNumberOfActions = 0;
            foreach (object folder in list)
            {
                    borderlist = parseKinectDataBorders(searchedLabel, folder.ToString());
                    foreach (object border in borderlist)
                    {
                        totalNumberOfActions++;
                    }
            }

            int[] actionSizes = new int[totalNumberOfActions];
            int actionNumber = 0; // file names for each action that is recorded in borders

            bool parsedBefore = false;

            // Create parsed folder if it doesnt exists
            bool exists = System.IO.Directory.Exists(subPath);
            if (!exists)
                System.IO.Directory.CreateDirectory(subPath);
            else
            {
                parsedBefore = true;
                Console.WriteLine("Data were parsed before! deleting prev");
                System.IO.Directory.Delete(subPath, true);
                System.IO.Directory.CreateDirectory(subPath);
            }
             
                foreach (object folder in list)
                {
                    int[] returnInfo = new int[actionNumber];
                        Console.WriteLine("Parsed folder: " + folder.ToString());
                        borderlist = parseKinectDataBorders(searchedLabel, folder.ToString());

                        foreach (object border in borderlist)
                        {
                            actionNumber++; // 1.txt 2.txt etc,,
                            Borders b = (Borders)border;
                            // Console.WriteLine(b.label.ToString() + " " + b.start.ToString() + " " + b.end.ToString());

                            coordinateList = parseKinectDataFiles(folder.ToString(), b);
                            // bir hareketin coordinat listesini aldik.
                            // bunu bir dosyaya 1.txt diye yaz. 

                            FileStream fs = new FileStream(subPath + "\\" + actionNumber.ToString() + ".csv", FileMode.Append, FileAccess.Write);
                            StreamWriter sw = new StreamWriter(fs);

                            int coordinateCount = 0;
                            foreach (object coord in coordinateList)
                            {
                                Coordinate c = (Coordinate)coord;
                                sw.Write(c.lx + ";" + c.ly + ";" + c.lz + ";");
                                sw.Write(c.rx + ";" + c.ry + ";" + c.rz + "\r\n");
                                coordinateCount++;
                            }
                            //Console.WriteLine("coordinateCount: " + coordinateCount);
                            sw.Flush();
                            sw.Close();
                            fs.Close();
                            actionSizes[actionNumber - 1] = coordinateCount;
                        } 
            } // Data parse done! actionNumber found

            return actionSizes;
        }

        private ArrayList GetSubDirectories(string root)
        {
            ArrayList list = new ArrayList();

            // Get all subdirectories
            string[] subdirectoryEntries = Directory.GetDirectories(root);
            // Loop through them to see if they have any other subdirectories
            foreach (string subdirectory in subdirectoryEntries)
            {
                list = LoadSubDirs(subdirectory, list);
            }
            return list;
        }
        private ArrayList LoadSubDirs(string dir, ArrayList list)
        {
            list.Add(dir);
            string[] subdirectoryEntries = Directory.GetDirectories(dir);
            foreach (string subdirectory in subdirectoryEntries)
            {
                list = LoadSubDirs(subdirectory, list);
            }
            return list;
        }

        private ArrayList parseKinectDataFiles(string dir, Borders b)
        {
            ArrayList coords = new ArrayList();
            TextReader tr = new StreamReader(dir + "\\body.csv");
            int lineCount = 0;
            lineCount++;

            while (lineCount != b.start)
            {
                tr.ReadLine();
                lineCount++;
            }

            while (lineCount != b.end)
            {
                string line = tr.ReadLine();
                lineCount++;
                string[] items = line.Split(';');
                Coordinate c = new Coordinate();

                c.lx = Double.Parse(items[93]);
                c.ly = Double.Parse(items[94]);
                c.lz = Double.Parse(items[95]);
                c.rx = Double.Parse(items[145]);
                c.ry = Double.Parse(items[146]);
                c.rz = Double.Parse(items[147]);

                coords.Add(c);
            }
            tr.Close();
            return coords;
        }

        private ArrayList parseKinectDataBorders(string searchedLabel, string dir)
        {
            ArrayList border = new ArrayList();
            StreamReader reader = File.OpenText(dir + "\\borders.csv");
            string line;
            while ((line = reader.ReadLine()) != null)
            {
                string[] items = line.Split(';');

                string label = items[0]; // sign label
                int start = int.Parse(items[1]); // borders of sign movement
                int end = int.Parse(items[2]); // borders of sign movement
                if (label.Equals(searchedLabel))
                {
                    Borders b = new Borders();
                    b.label = label;
                    b.start = start;
                    b.end = end;
                    border.Add(b);
                }
            }
            return border;
        } // end of parse data border
    }

    public class Person
    {
        public string name;
        public Boolean isSick;
        public Boolean isSeekingInfo;
        public Boolean isEmergency;
        public Boolean hasAppointment;
        public string otherPatientName;
        public string grievance;
        public string soughtInfo;
        public string programFlow;

        public Person()
        {
            this.name = "";
            this.otherPatientName = "";
            this.grievance = "";
            this.soughtInfo = "";
            this.programFlow = "";
            this.isEmergency = false;
            this.isSeekingInfo = false;
            this.isSick = false;
            this.hasAppointment = false;
        }

        public void printOut() {
            String fileName = string.Format(@"{0}.txt", Guid.NewGuid());
            String filePath = "C:/Users/hilal/Desktop/Printouts/"+fileName;
            if (System.IO.File.Exists(filePath) == true)
            {
                FileStream fs = new FileStream(filePath, FileMode.Append, FileAccess.Write);
                StreamWriter objWrite = new StreamWriter(fs);
                objWrite.WriteLine("Kisi Bilgileri:");
                if (this.isSeekingInfo) {
                    objWrite.WriteLine("Bu kisi bilgi istiyor.");
                    objWrite.WriteLine("Istedigi bilgi: "+this.soughtInfo);
                    if (this.soughtInfo.Equals("Hasta Ziyareti İçin Geldim"))
                    {
                        objWrite.WriteLine("Ziyaret etmek istedigi hastanin ismi: " + this.otherPatientName);
                    }

                }
                else if (this.isSick) {
                    objWrite.WriteLine("Bu kisi hasta.");
                    if (this.isEmergency)
                    {
                        objWrite.WriteLine("Durumu acil.");
                    }
                    else {
                        objWrite.WriteLine("Durumu acil degil.");
                    }
                    objWrite.WriteLine("Sikayeti: "+this.grievance);
                    if (!this.isEmergency) {
                        if (this.hasAppointment)
                        {
                            objWrite.WriteLine("Rendevusu var.");
                        }
                        else {
                            objWrite.WriteLine("Randevusu yok.");
                        }
                    }
                    objWrite.WriteLine("Hastanin ismi: "+this.name);


                }
                objWrite.WriteLine("Program Akisi:");
                objWrite.Write(this.programFlow);
                objWrite.Close();
            }
            else
            {
                FileStream fs = new FileStream(filePath, FileMode.Create, FileAccess.Write);
                StreamWriter objWrite = new StreamWriter(fs);
                objWrite.WriteLine("Kisi Bilgileri:");
                if (this.isSeekingInfo)
                {
                    objWrite.WriteLine("Bu kisi bilgi istiyor.");
                    objWrite.WriteLine("Istedigi bilgi: " + this.soughtInfo);
                    if (this.soughtInfo.Equals("Hasta Ziyareti İçin Geldim"))
                    {
                        objWrite.WriteLine("Ziyaret etmek istedigi hastanin ismi: " + this.otherPatientName);
                    }

                }
                else if (this.isSick)
                {
                    objWrite.WriteLine("Bu kisi hasta.");
                    if (this.isEmergency)
                    {
                        objWrite.WriteLine("Durumu acil.");
                    }
                    else
                    {
                        objWrite.WriteLine("Durumu acil degil.");
                    }
                    objWrite.WriteLine("Sikayeti: " + this.grievance);
                    if (!this.isEmergency)
                    {
                        if (this.hasAppointment)
                        {
                            objWrite.WriteLine("Rendevusu var.");
                        }
                        else
                        {
                            objWrite.WriteLine("Randevusu yok.");
                        }
                    }
                    objWrite.WriteLine("Hastanin ismi: " + this.name);


                }
                objWrite.WriteLine("Program Akisi:");
                objWrite.Write(this.programFlow);
                objWrite.Close();
            }
        }
    }
}
