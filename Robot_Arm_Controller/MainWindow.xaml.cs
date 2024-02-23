#define IRB6700

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.IO.Ports;
using HelixToolkit.Wpf;
using System.IO;
using System.ComponentModel;
using System.Reflection;
using System.Threading;
using System.Windows.Forms;
using System.Windows.Markup;

namespace Robot_Arm_Controller
{
    class Joint
    {
        public Model3D model = null;
        public double angle = 0;
        public double angleMin = -180;
        public double angleMax = 180;
        public double stepPerDegree = 1;
        public double realAngle = 0;
        public int rotPointX = 0;
        public int rotPointY = 0;
        public int rotPointZ = 0;
        public int rotAxisX = 0;
        public int rotAxisY = 0;
        public int rotAxisZ = 0;

        public Joint(Model3D pModel)
        {
            model = pModel;
        }
    }


    public partial class MainWindow : Window
    {
        //provides functionality to 3d models
        Model3DGroup RA = new Model3DGroup(); //RoboticArm 3d group
        Model3D geom = null; //Debug sphere to check in which point the joint is rotatin

        List<Joint> joints = null;

        BackgroundWorker bgw = new BackgroundWorker();
        BackgroundWorker modelUpdater = new BackgroundWorker();
        bool isAnimateMission = false;
        bool isModelUpdater = false;

        SerialPort sp = new SerialPort();

        bool switchingJoint = false;
        bool isAnimating = false;

        Color oldColor = Colors.White;
        GeometryModel3D oldSelectedModel = null;
        string basePath = "";
        ModelVisual3D visual;
        double LearningRate = 0.01;
        double SamplingDistance = 0.15;
        double DistanceThreshold = 20;
        //provides render to model3d objects
        ModelVisual3D RoboticArm = new ModelVisual3D();
        Transform3DGroup F1;
        Transform3DGroup F2;
        Transform3DGroup F3;
        Transform3DGroup F4;
        Transform3DGroup F5;
        Transform3DGroup F6;
        Transform3DGroup F7;
        ///Transform3DGroup F8;
        RotateTransform3D R;
        TranslateTransform3D T;
        Vector3D reachingPoint;
        int movements = 10;
        // Timer
        System.Windows.Forms.Timer timer1;


#if IRB6700
        //directroy of all stl files
        private const string MODEL_PATH8 = "J0.stl";
        private const string MODEL_PATH1 = "J1.stl";
        private const string MODEL_PATH2 = "J2.stl";
        private const string MODEL_PATH3 = "J3.stl";
        private const string MODEL_PATH4 = "J4.stl";
        private const string MODEL_PATH5 = "J5.stl";
        private const string MODEL_PATH6 = "GripperRight.stl";
        private const string MODEL_PATH7 = "GripperLeft.stl";
#else
        private const string MODEL_PATH1 = "J0.stl";
        private const string MODEL_PATH2 = "J1.stl";
        private const string MODEL_PATH3 = "J2.stl";
        private const string MODEL_PATH4 = "J3.stl";
        private const string MODEL_PATH5 = "J4.stl";
        private const string MODEL_PATH6 = "J5.stl";
        private const string MODEL_PATH7 = "GripperRight.stl";
        private const string MODEL_PATH8 = "GripperLeft.stl";
#endif

        public MainWindow()
        {
            InitializeComponent();
            // Give the path of the folder where the .stl files are stored
            basePath = "C:\\Users\\omerg\\source\\repos\\Robot_Arm_Controller\\Robot_Arm_Controller\\STLFiles\\";
            //basePath = Directory.GetParent(Directory.GetCurrentDirectory()).Parent.FullName + "\\STLFiles\\";
            List<string> modelsNames = new List<string>();
            modelsNames.Add(MODEL_PATH1);
            modelsNames.Add(MODEL_PATH2);
            modelsNames.Add(MODEL_PATH3);
            modelsNames.Add(MODEL_PATH4);
            modelsNames.Add(MODEL_PATH5);
            modelsNames.Add(MODEL_PATH6);
            modelsNames.Add(MODEL_PATH7);
            modelsNames.Add(MODEL_PATH8);

            RoboticArm.Content = Initialize_Environment(modelsNames);

            /** Debug sphere to check in which point the joint is rotating**/
            var builder = new MeshBuilder(true, true);
            var position = new Point3D(0, 0, 0);
            builder.AddSphere(position, 50, 15, 15);
            geom = new GeometryModel3D(builder.ToMesh(), Materials.Brown);
            visual = new ModelVisual3D();
            visual.Content = geom;

            viewPort3d.RotateGesture = new MouseGesture(MouseAction.RightClick);
            viewPort3d.PanGesture = new MouseGesture(MouseAction.LeftClick);
            viewPort3d.Children.Add(visual);
            viewPort3d.Children.Add(RoboticArm);
            viewPort3d.Camera.LookDirection = new Vector3D(12270, -12270, -12270);
            viewPort3d.Camera.UpDirection = new Vector3D(0, 0, 1);
            viewPort3d.Camera.Position = new Point3D(-11290, 11375, 13650);

            double[] angles = { joints[0].angle, joints[1].angle, joints[2].angle, joints[3].angle, joints[4].angle, joints[5].angle, joints[6].angle };
            ForwardKinematics(angles);

            changeSelectedJoint();

            timer1 = new System.Windows.Forms.Timer();

            //Seri port nesnesi oluştur
            

            timer1.Interval = 5;
            timer1.Tick += new System.EventHandler(timer1_Tick);
            
            modelUpdater.DoWork += (s, e) =>
            {
                while (isModelUpdater)
                {
                    bool isDifferent = false;
                    // real angle ile value faklı ise güncelle
                    Dispatcher.Invoke(() => 
                    {
                        bool isDifferent = joints[0].realAngle != joint1.Value || joints[1].realAngle != joint2.Value || joints[2].realAngle != joint3.Value || joints[3].realAngle != joint4.Value || joints[4].realAngle != joint5.Value || joints[5].realAngle != joint6.Value;
                        });
                    if (!isDifferent)
                    {
                        Dispatcher.Invoke(() =>
                        {
                            joint1.Value = joints[0].realAngle;
                            joint2.Value = joints[1].realAngle;
                            joint3.Value = joints[2].realAngle;
                            joint4.Value = joints[3].realAngle;
                            joint5.Value = joints[4].realAngle;
                            joint6.Value = joints[5].realAngle;
                            //joint7.Value = joints[6].realAngle;
                            //joint8.Value = joints[7].realAngle;
                        });
                    }

                    Thread.Sleep(100);
                }
            };  

            modelUpdater.RunWorkerCompleted += (s, e) =>
            {
                isModelUpdater = false;
            };
            
            bgw.DoWork += (s, e) =>
            {
                isAnimateMission = true;
                // gorevSimulasyon metodunun içeriğini buraya taşı
                bgw.WorkerSupportsCancellation = true;

                string[] gorevler = new string[gorevList.Items.Count];
                for (int i = 0; i < gorevList.Items.Count; i++)
                {
                    gorevler[i] = gorevList.Items[i].ToString();
                }
                int gorevSayisi = gorevList.Items.Count;
                int sayac = 0;
                //
                while (sayac < gorevSayisi-1)
                {
                    string data = gorevler[sayac];
                    string dataNext = gorevler[sayac + 1];
                    string[] degerler = data.Split('/');
                    string[] degerlerNext = dataNext.Split('/');
                    double j1 = Convert.ToDouble(degerler[0]);
                    double j2 = Convert.ToDouble(degerler[1]);
                    double j3 = Convert.ToDouble(degerler[2]);
                    double j4 = Convert.ToDouble(degerler[3]);
                    double j5 = Convert.ToDouble(degerler[4]);
                    double j1Next = Convert.ToDouble(degerlerNext[0]);
                    double j2Next = Convert.ToDouble(degerlerNext[1]);
                    double j3Next = Convert.ToDouble(degerlerNext[2]);
                    double j4Next = Convert.ToDouble(degerlerNext[3]);
                    double j5Next = Convert.ToDouble(degerlerNext[4]);
                    double dj1 = (j1Next - j1);
                    double dj2 = (j2Next - j2);
                    double dj3 = (j3Next - j3);
                    double dj4 = (j4Next - j4);
                    double dj5 = (j5Next - j5);

                    //her döngüde kaç ms bekleyeceğini bul
                    double j1Step = dj1 / 100;
                    double j2Step = dj2 / 100;
                    double j3Step = dj3 / 100;
                    double j4Step = dj4 / 100;
                    double j5Step = dj5 / 100;

                    int step = 0;
                    while(step < 100)
                    {
                        if (bgw.CancellationPending)
                        {
                            // İptal edildiyse, işlemi durdur
                            e.Cancel = true;
                            return;
                        }
                        Dispatcher.Invoke(() =>
                        {
                            joint1.Value = j1;
                            joint2.Value = j2;
                            joint3.Value = j3;
                            joint4.Value = j4;
                            joint5.Value = j5;
                            
                        });
                        j1 += j1Step;
                        j2 += j2Step;
                        j3 += j3Step;
                        j4 += j4Step;
                        j5 += j5Step;
                        Thread.Sleep(10);
                        step++;
                    }


                    // UI thread'inde çalışan değişkenleri güncellemek için
                    // Dispatcher.Invoke metodunu kullan

                    sayac++;
                }
            };

            // RunWorkerCompleted event handler'ını tanımla
            // Bu metot, arka plandaki işlem bittiğinde çalışır
            bgw.RunWorkerCompleted += (s, e) =>
            {
                isAnimateMission = false;
                // İşlem bittiğinde yapmak istediğiniz şeyleri buraya yazın
                // Örneğin, bir mesaj kutusu göster
                btnG4.Content = "Görev Önizleme";
                // Butonun rengini #FF008CC8 yap
                btnG4.Background = new SolidColorBrush(ColorHelper.HexToColor("#FF008CC8"));
                //System.Windows.MessageBox.Show("Görev simülasyonu tamamlandı.");
            };
        }

        // Seri porttaki nesneleri tarayıp verileri combox'a ekleeyen buton
        private void btnPort_Click(object sender, RoutedEventArgs e)
        {
            // Seri port adlarını al
            string[] portNames = SerialPort.GetPortNames();

            // Seri port adlarını comboBox'a ekle
            comboBox.ItemsSource = portNames;
        }

        private void baglanButon(object sender, RoutedEventArgs e)
        {
            // Seri port nesnesi oluştur
            

            // Seri port ayarlarını yap
            //comboBox boş olup olmadığını kontrol et
            if (comboBox.Text == "")
            {
                System.Windows.MessageBox.Show("Lütfen bir seri port seçiniz.");
                return;
            }
            sp.PortName = comboBox.Text;
            sp.BaudRate = 9600;
            sp.Parity = Parity.None;
            sp.DataBits = 8;
            sp.StopBits = StopBits.One;

            // Seri portu aç
            try
            {
                sp.Open();
                baglanBtn.IsEnabled = false;
                baglantiKesBtn.IsEnabled = true;
                durumAyarla(1);
                sp.DataReceived += new SerialDataReceivedEventHandler(sp_DataReceived);
            }
            catch (Exception exc)
            {
                System.Windows.MessageBox.Show("Seri port açılamadı.");

            }
        }

        void durumAyarla(int state)
        {
            if (state == 0) // Baglantı Yok
            {
                stateBorder.Background = new SolidColorBrush(ColorHelper.HexToColor("#FFC6423A"));
                stateLabel.Content = "Bağlanti Yok";
            }
            else if (state == 1) // Hazır
            {
                stateBorder.Background = new SolidColorBrush(ColorHelper.HexToColor("#FF46C800"));
                stateLabel.Content = "      Hazır";
            }
            else if (state == 2) // Hareket Ediyor
            {
                stateBorder.Background = new SolidColorBrush(ColorHelper.HexToColor("#EFD200"));
                stateLabel.Content = "   Çalışıyor";
            }
            else if (state == 3) // Hata
            {

            }
        }

        private void baglantiKesButon(object sender, RoutedEventArgs e)
        {

            // Seri portu kapat
            try
            {
                sp.Close();
                baglanBtn.IsEnabled = true;
                baglantiKesBtn.IsEnabled = false;
                durumAyarla(0);

            }
            catch (Exception exc)
            {
                System.Windows.MessageBox.Show("Seri port kapatılamadı.");

            }
        }

        ////Seri porttan veri geldiğinde tetiklenen fonksiyon
        private void sp_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            // Seri porttan gelen veriyi oku
            string data = sp.ReadLine();
            //sp.DiscardInBuffer();
            //datanın ilk elemanı < ise ve son elemanı > ise

            //datanın içindeki < ve > işaretinin arasındaki veriyi al ve mesaj değişkenine ata
            string message = ""; // Mesajı tutacak string
            // < ve > işaretlerinin arasındaki veriyi al
            foreach (char c in data)
            {
                if (c == '<')
                {
                    message = "";
                }
                else if (c == '>')
                {
                    break;
                }
                else
                {
                    message += c;
                }
            }
            if (message != "")
            {
                // Seri porttan gelen veriyi ekrana yaz
                Dispatcher.Invoke(() =>
                {
                    veriAnaliz(message);
                    logYaz(message);
                });
            }
            else
            {
                   // < ve > işaretleri yoksa veri geçersizdir
                return;
            }

        }

        //private void sp_DataReceived(object sender, SerialDataReceivedEventArgs e)
        //{
        //    // Seri port nesnesini al
        //    //SerialPort sp = (SerialPort)sender;

        //    // Veriyi oku
        //    string message = ""; // Mesajı tutacak string
        //    bool start = false; // Verinin başladığını belirten bayrak
        //    bool end = false; // Verinin bittiğini belirten bayrak
        //    while (!end) // Veri bitene kadar döngüyü sürdür
        //    {
        //        int c = sp.ReadChar(); // Bir karakter oku
        //        if (c == '<') // Karakter başlangıç işareti ise
        //        {
        //            start = true; // Bayrağı kaldır
        //            //message += (char)c;
        //        }
        //        else if (c == '>') // Karakter bitiş işareti ise
        //        {
        //            end = true; // Bayrağı kaldır
        //            //message += (char)c;
        //        }
        //        else if (start) // Karakter mesajın bir parçası ise
        //        {
        //            message += (char)c; // Stringe ekle
        //        }
        //    }

        //    // Veriyi ana iş parçacığına gönder
        //    Dispatcher.Invoke(() =>
        //    {
        //        veriAnaliz(message);
        //        logYaz(message);
        //    });
        //}

        //Seri porttan gelen veriyi analiz eden fonksiyon
        private void veriAnaliz(string data)
        {
            // Veri boş veya null ise dön
            if (string.IsNullOrEmpty(data))
            {
                return;
            }
            if (data == "1")
            {
                durumAyarla(1);
                return;
            }
            else if (data == "2")
            {
                durumAyarla(2); // çalışıyor
                return;
            }

            // Veriyi , işaretine göre parçala
            string[] degerler = data.Split(',');
            // Verinin ilk elemanı POZ ise
            if (degerler != null && degerler[0] == "POZ")
            {
                // Verinin ikinci ve üçüncü elemanlarını int türüne dönüştür
                int no = Convert.ToInt32(degerler[1]);
                int deger = Convert.ToInt32(degerler[2]);
                // No ya göre pozisyonu değiştir
                joints[no].realAngle = deger / joints[no].stepPerDegree;
                updateRealAngles();
                return;
            }
            
            else
            {
                // Veri geçersiz ise hata mesajı göster
                logYaz("Geçersiz veri geldi: " + data);
            }
        }

        //gerçek açıları değiştiren fonksiyon
        private void updateRealAngles()
        {

            rJ1.Content = joints[0].realAngle.ToString();
            rJ2.Content = joints[1].realAngle.ToString();
            rJ3.Content = joints[2].realAngle.ToString();
            rJ4.Content = joints[3].realAngle.ToString();
            rJ5.Content = joints[4].realAngle.ToString();
            rJ6.Content = joints[5].realAngle.ToString();

            //joint1.Value = joints[0].realAngle;
            //joint2.Value = joints[1].realAngle;
            //joint3.Value = joints[2].realAngle;
            //joint4.Value = joints[3].realAngle;
            //joint5.Value = joints[4].realAngle;
            //joint6.Value = joints[5].realAngle;
        }

        //Logları yazdıran fonksiyon, ilk önce saati alıyor, sonra logBox'a yazıyor
        private void logYaz(string log)
        {
            //Yazdıkça aşağı kaydır
            string saat = DateTime.Now.ToString("HH:mm:ss");
            logBox.Text += "[" + saat + "] " + log + "\n";
            logBox.ScrollToEnd();

        }


        private Model3DGroup Initialize_Environment(List<string> modelsNames)
        {
            try
            {
                ModelImporter import = new ModelImporter();
                joints = new List<Joint>();

                foreach (string modelName in modelsNames)
                {
                    var materialGroup = new MaterialGroup();
                    Color mainColor = Colors.White;
                    EmissiveMaterial emissMat = new EmissiveMaterial(new SolidColorBrush(mainColor));
                    DiffuseMaterial diffMat = new DiffuseMaterial(new SolidColorBrush(mainColor));
                    SpecularMaterial specMat = new SpecularMaterial(new SolidColorBrush(mainColor), 200);
                    materialGroup.Children.Add(emissMat);
                    materialGroup.Children.Add(diffMat);
                    materialGroup.Children.Add(specMat);

                    var link = import.Load(basePath + modelName);
                    GeometryModel3D model = link.Children[0] as GeometryModel3D;
                    model.Material = materialGroup;
                    model.BackMaterial = materialGroup;
                    joints.Add(new Joint(link));
                }

                RA.Children.Add(joints[0].model);
                RA.Children.Add(joints[1].model);
                RA.Children.Add(joints[2].model);
                RA.Children.Add(joints[3].model);
                RA.Children.Add(joints[4].model);
                RA.Children.Add(joints[5].model);
                RA.Children.Add(joints[6].model);
                RA.Children.Add(joints[7].model);

                changeModelColor(joints[0], ColorHelper.HexToColor("#FF46C800"));
                changeModelColor(joints[1], ColorHelper.HexToColor("#FF46C800"));
                changeModelColor(joints[2], ColorHelper.HexToColor("#FF46C800"));
                changeModelColor(joints[3], ColorHelper.HexToColor("#FF46C800"));
                changeModelColor(joints[4], ColorHelper.HexToColor("#FF46C800"));
                changeModelColor(joints[5], ColorHelper.HexToColor("#FF46C800"));
                changeModelColor(joints[6], ColorHelper.HexToColor("#FF46C800"));
                changeModelColor(joints[7], ColorHelper.HexToColor("#1D8348"));

                //Gövde (Bu hareket etmeyecek)
                //joints[0].angleMin = -180;
                //joints[0].angleMax = 180;
                //joints[0].rotAxisX = 0;
                //joints[0].rotAxisY = 0;
                //joints[0].rotAxisZ = 1;
                //joints[0].rotPointX = 0;
                //joints[0].rotPointY = 0;
                //joints[0].rotPointZ = 0;

                //Omuz (Bu hareket edecek) (Z ekseninde)
                joints[0].stepPerDegree =37.5;
                joints[0].angleMin = -90;
                joints[0].angleMax = 90;
                joints[0].rotAxisX = 0;
                joints[0].rotAxisY = 0;
                joints[0].rotAxisZ = 1;
                joints[0].rotPointX = 0;
                joints[0].rotPointY = 0;
                joints[0].rotPointZ = 650;

                //Dirsek (Bu hareket edecek) (X ekseninde)
                joints[1].stepPerDegree = 40;
                joints[1].angleMin = -90;
                joints[1].angleMax = 90;
                joints[1].rotAxisX = 1;
                joints[1].rotAxisY = 0;
                joints[1].rotAxisZ = 0;
                joints[1].rotPointX = 0;
                joints[1].rotPointY = 0;
                joints[1].rotPointZ = 2300;

                //Bilek (Bu hareket edecek) (Z ekseninde)
                joints[2].stepPerDegree = 100;
                joints[2].angleMin = -100;
                joints[2].angleMax = 100;
                joints[2].rotAxisX = 1;
                joints[2].rotAxisY = 0;
                joints[2].rotAxisZ = 0;
                joints[2].rotPointX = 0;
                joints[2].rotPointY = 0;
                joints[2].rotPointZ = 4500;

                joints[3].stepPerDegree = 2;
                joints[3].angleMin = -120;
                joints[3].angleMax = 120;
                joints[3].rotAxisX = 0;
                joints[3].rotAxisY = 0;
                joints[3].rotAxisZ = 1;
                joints[3].rotPointX = 0;
                joints[3].rotPointY = 0;
                joints[3].rotPointZ = 5775;

                joints[4].stepPerDegree = 20;
                joints[4].angleMin = -90;
                joints[4].angleMax = 90;
                joints[4].rotAxisX = 1;
                joints[4].rotAxisY = 0;
                joints[4].rotAxisZ = 0;
                joints[4].rotPointX = 0;
                joints[4].rotPointY = 0;
                joints[4].rotPointZ = 6725;

                joints[5].angleMin = -30;
                joints[5].angleMax = 0;
                joints[5].rotAxisX = 0;
                joints[5].rotAxisY = 1;
                joints[5].rotAxisZ = 0;
                joints[5].rotPointX = 0;
                joints[5].rotPointY = -140;
                joints[5].rotPointZ = 7225;

                joints[6].angleMin = -30;
                joints[6].angleMax = 0;
                joints[6].rotAxisX = 0;
                joints[6].rotAxisY = 1;
                joints[6].rotAxisZ = 0;
                joints[6].rotPointX = 0;
                joints[6].rotPointY = 140;
                joints[6].rotPointZ = 7225;

            }
            catch (Exception e)
            {
                System.Windows.MessageBox.Show("Exception Error:" + e.StackTrace);
            }
            return RA;
        }

        public static T Clamp<T>(T value, T min, T max)
            where T : System.IComparable<T>
        {
            T result = value;
            if (value.CompareTo(max) > 0)
                result = max;
            if (value.CompareTo(min) < 0)
                result = min;
            return result;
        }

        private void ReachingPoint_TextChanged(object sender, TextChangedEventArgs e)
        {
            try
            {
                reachingPoint = new Vector3D(Double.Parse(TbX.Text), Double.Parse(TbY.Text), Double.Parse(TbZ.Text));
                geom.Transform = new TranslateTransform3D(reachingPoint);
            }
            catch (Exception exc)
            {

            }
        }

        private void jointSelector_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            changeSelectedJoint();
        }
        private void realJointSelector_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            
        }

        private void changeSelectedJoint()
        {
            if (joints == null)
                return;

            int sel = ((int)jointSelector.Value) - 1;
            switchingJoint = true;
            unselectModel();
            if (sel < 0)
            {
                jointX.IsEnabled = false;
                jointY.IsEnabled = false;
                jointZ.IsEnabled = false;
                jointXAxis.IsEnabled = false;
                jointYAxis.IsEnabled = false;
                jointZAxis.IsEnabled = false;
            }
            else
            {
                if (!jointX.IsEnabled)
                {
                    jointX.IsEnabled = true;
                    jointY.IsEnabled = true;
                    jointZ.IsEnabled = true;
                    jointXAxis.IsEnabled = true;
                    jointYAxis.IsEnabled = true;
                    jointZAxis.IsEnabled = true;
                }
                jointX.Value = joints[sel].rotPointX;
                jointY.Value = joints[sel].rotPointY;
                jointZ.Value = joints[sel].rotPointZ;
                jointXAxis.IsChecked = joints[sel].rotAxisX == 1 ? true : false;
                jointYAxis.IsChecked = joints[sel].rotAxisY == 1 ? true : false;
                jointZAxis.IsChecked = joints[sel].rotAxisZ == 1 ? true : false;
                selectModel(joints[sel].model);
                updateSpherePosition();
            }
            switchingJoint = false;
        }

        private void rotationPointChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (switchingJoint)
                return;

            int sel = ((int)jointSelector.Value) - 1;
            joints[sel].rotPointX = (int)jointX.Value;
            joints[sel].rotPointY = (int)jointY.Value;
            joints[sel].rotPointZ = (int)jointZ.Value;
            updateSpherePosition();
        }

        private void updateSpherePosition()
        {
            int sel = ((int)jointSelector.Value) - 1;
            if (sel < 0)
                return;

            Transform3DGroup F = new Transform3DGroup();
            F.Children.Add(new TranslateTransform3D(joints[sel].rotPointX, joints[sel].rotPointY, joints[sel].rotPointZ));
            F.Children.Add(joints[sel].model.Transform);
            geom.Transform = F;
        }

        private void button_Click(object sender, RoutedEventArgs e)
        {

        }

        private void CheckBox_StateChanged(object sender, RoutedEventArgs e)
        {
            if (switchingJoint)
                return;

            int sel = ((int)jointSelector.Value) - 1;
            joints[sel].rotAxisX = jointXAxis.IsChecked.Value ? 1 : 0;
            joints[sel].rotAxisY = jointYAxis.IsChecked.Value ? 1 : 0;
            joints[sel].rotAxisZ = jointZAxis.IsChecked.Value ? 1 : 0;
        }

        private void joint_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (isAnimating)
                return;

            joints[0].angle = joint1.Value;
            joints[1].angle = joint2.Value;
            joints[2].angle = joint3.Value;
            joints[3].angle = joint4.Value;
            joints[4].angle = joint5.Value;
            joints[5].angle = joint6.Value;
            joints[6].angle = -joint6.Value;
            execute_fk();
        }
        private void execute_fk()
        {
            /** Debug sphere, it takes the x,y,z of the textBoxes and update its position
             * This is useful when using x,y,z in the "new Point3D(x,y,z)* when defining a new RotateTransform3D() to check where the joints is actually  rotating */
            double[] angles = { joints[0].angle, joints[1].angle, joints[2].angle, joints[3].angle, joints[4].angle, joints[5].angle, joints[6].angle };
            ForwardKinematics(angles);
            updateSpherePosition();
        }


        private Color changeModelColor(Joint pJoint, Color newColor)
        {
            Model3DGroup models = ((Model3DGroup)pJoint.model);
            return changeModelColor(models.Children[0] as GeometryModel3D, newColor);
        }

        private Color changeModelColor(GeometryModel3D pModel, Color newColor)
        {
            if (pModel == null)
                return oldColor;

            Color previousColor = Colors.Black;

            MaterialGroup mg = (MaterialGroup)pModel.Material;
            if (mg.Children.Count > 0)
            {
                try
                {
                    previousColor = ((EmissiveMaterial)mg.Children[0]).Color;
                    ((EmissiveMaterial)mg.Children[0]).Color = newColor;
                    ((DiffuseMaterial)mg.Children[1]).Color = newColor;
                }
                catch (Exception exc)
                {
                    previousColor = oldColor;
                }
            }

            return previousColor;
        }

        private void selectModel(Model3D pModel)
        {
            try
            {
                Model3DGroup models = ((Model3DGroup)pModel);
                oldSelectedModel = models.Children[0] as GeometryModel3D;
            }
            catch (Exception exc)
            {
                oldSelectedModel = (GeometryModel3D)pModel;
            }
            oldColor = changeModelColor(oldSelectedModel, ColorHelper.HexToColor("#ff3333"));
        }

        private void unselectModel()
        {
            changeModelColor(oldSelectedModel, oldColor);
        }

        private void ViewPort3D_OnMouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            Point mousePos = e.GetPosition(viewPort3d);
            PointHitTestParameters hitParams = new PointHitTestParameters(mousePos);
            VisualTreeHelper.HitTest(viewPort3d, null, ResultCallback, hitParams);
        }

        private void ViewPort3D_OnMouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            // Perform the hit test on the mouse's position relative to the viewport.
            HitTestResult result = VisualTreeHelper.HitTest(viewPort3d, e.GetPosition(viewPort3d));
            RayMeshGeometry3DHitTestResult mesh_result = result as RayMeshGeometry3DHitTestResult;

            if (oldSelectedModel != null)
                unselectModel();

            if (mesh_result != null)
            {
                selectModel(mesh_result.ModelHit);
            }
        }

        public HitTestResultBehavior ResultCallback(HitTestResult result)
        {
            // Did we hit 3D?
            RayHitTestResult rayResult = result as RayHitTestResult;
            if (rayResult != null)
            {
                // Did we hit a MeshGeometry3D?
                RayMeshGeometry3DHitTestResult rayMeshResult = rayResult as RayMeshGeometry3DHitTestResult;
                geom.Transform = new TranslateTransform3D(new Vector3D(rayResult.PointHit.X, rayResult.PointHit.Y, rayResult.PointHit.Z));

                if (rayMeshResult != null)
                {
                    // Yes we did!
                }
            }

            return HitTestResultBehavior.Continue;
        }

        public void StartInverseKinematics(object sender, RoutedEventArgs e)
        {
            if (timer1.Enabled)
            {
                inverseStart.Content = "Pozisyona Git";
                // Butonun rengini #FF008CC8 yap
                inverseStart.Background = new SolidColorBrush(ColorHelper.HexToColor("#FF008CC8"));

                isAnimating = false;
                timer1.Stop();
                movements = 0;
                
            }
            else
            {
                geom.Transform = new TranslateTransform3D(reachingPoint);
                movements = 5000;
                inverseStart.Content = "Durdur";
                // Butonun rengini #FFC6423A yap
                inverseStart.Background = new SolidColorBrush(ColorHelper.HexToColor("#FFC6423A"));
                isAnimating = true;
                timer1.Start();
            }
        }

        private void hizIvmeAyarla(object sender, RoutedEventArgs e)
        {
            int motorNo = Convert.ToInt32(realJointSelector.Value)-1;
            if (motorNo < 0 || motorNo > 4)
                return;
            int ivme = Convert.ToInt32(ivmeAyar.Text);
            int hiz = Convert.ToInt32(hizAyar.Text);
            string data = "<HIZIVME," + motorNo + "," + hiz + "," + ivme + ",>";
            sp.Write(data);

        }

        public void timer1_Tick(object sender, EventArgs e)
        {
            double[] angles = { joints[0].angle, joints[1].angle, joints[2].angle, joints[3].angle, joints[4].angle, joints[5].angle, joints[6].angle };
            angles = InverseKinematics(reachingPoint, angles);
            joint1.Value = joints[0].angle = angles[0];
            joint2.Value = joints[1].angle = angles[1];
            joint3.Value = joints[2].angle = angles[2];
            joint4.Value = joints[3].angle = angles[3];
            joint5.Value = joints[4].angle = angles[4];
            joint6.Value = joints[5].angle = angles[5];
            //joint7.Value = joints[6].angle = angles[6];
            //joint8.Value = joints[7].angle = angles[7];

            if ((--movements) <= 0)
            {
                inverseStart.Content = "Pozisyona Git";
                // Butonun rengini #FF008CC8 yap
                inverseStart.Background = new SolidColorBrush(ColorHelper.HexToColor("#FF008CC8"));

                isAnimating = false;
                timer1.Stop();
            }
        }

        public double[] InverseKinematics(Vector3D target, double[] angles)
        {
            if (DistanceFromTarget(target, angles) < DistanceThreshold)
            {
                movements = 0;
                return angles;
            }

            double[] oldAngles = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
            angles.CopyTo(oldAngles, 0);
            for (int i = 0; i <= 5; i++)
            {
                // Gradient descent
                // Update : Solution -= LearningRate * Gradient
                double gradient = PartialGradient(target, angles, i);
                angles[i] -= LearningRate * gradient;

                // Clamp
                angles[i] = Clamp(angles[i], joints[i].angleMin, joints[i].angleMax);

                // Early termination
                if (DistanceFromTarget(target, angles) < DistanceThreshold || checkAngles(oldAngles, angles))
                {
                    movements = 0;
                    return angles;
                }
            }

            return angles;
        }

        public bool checkAngles(double[] oldAngles, double[] angles)
        {
            for (int i = 0; i <= 5; i++)
            {
                if (oldAngles[i] != angles[i])
                    return false;
            }

            return true;
        }

        public double PartialGradient(Vector3D target, double[] angles, int i)
        {
            // Saves the angle,
            // it will be restored later
            double angle = angles[i];

            // Gradient : [F(x+SamplingDistance) - F(x)] / h
            double f_x = DistanceFromTarget(target, angles);

            angles[i] += SamplingDistance;
            double f_x_plus_d = DistanceFromTarget(target, angles);

            double gradient = (f_x_plus_d - f_x) / SamplingDistance;

            // Restores
            angles[i] = angle;

            return gradient;
        }


        public double DistanceFromTarget(Vector3D target, double[] angles)
        {
            Vector3D point = ForwardKinematics(angles);
            return Math.Sqrt(Math.Pow((point.X - target.X), 2.0) + Math.Pow((point.Y - target.Y), 2.0) + Math.Pow((point.Z - target.Z), 2.0));
        }


        public Vector3D ForwardKinematics(double[] angles)
        {
            //The base only has rotation and is always at the origin, so the only transform in the transformGroup is the rotation R
            F1 = new Transform3DGroup();
            R = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(joints[0].rotAxisX, joints[0].rotAxisY, joints[0].rotAxisZ), angles[0]), new Point3D(joints[0].rotPointX, joints[0].rotPointY, joints[0].rotPointZ));
            F1.Children.Add(R);

            //This moves the first joint attached to the base, it may translate and rotate. Since the joint are already in the right position (the .stl model also store the joints position
            //in the virtual world when they were first created, so if you load all the .stl models of the joint they will be automatically positioned in the right locations)
            //so in all of these cases the first translation is always 0, I just left it for future purposes if something need to be moved
            //After that, the joint needs to rotate of a certain amount (given by the value in the slider), and the rotation must be executed on a specific point
            //After some testing it looks like the point 175, -200, 500 is the sweet spot to achieve the rotation intended for the joint
            //finally we also need to apply the transformation applied to the base 
            F2 = new Transform3DGroup();
            T = new TranslateTransform3D(0, 0, 0);
            R = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(joints[1].rotAxisX, joints[1].rotAxisY, joints[1].rotAxisZ), angles[1]), new Point3D(joints[1].rotPointX, joints[1].rotPointY, joints[1].rotPointZ));
            F2.Children.Add(T);
            F2.Children.Add(R);
            F2.Children.Add(F1);

            //The second joint is attached to the first one. As before I found the sweet spot after testing, and looks like is rotating just fine. No pre-translation as before
            //and again the previous transformation needs to be applied
            F3 = new Transform3DGroup();
            T = new TranslateTransform3D(0, 0, 0);
            R = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(joints[2].rotAxisX, joints[2].rotAxisY, joints[2].rotAxisZ), angles[2]), new Point3D(joints[2].rotPointX, joints[2].rotPointY, joints[2].rotPointZ));
            F3.Children.Add(T);
            F3.Children.Add(R);
            F3.Children.Add(F2);

            //as before
            F4 = new Transform3DGroup();
            T = new TranslateTransform3D(0, 0, 0); //1500, 650, 1650
            R = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(joints[3].rotAxisX, joints[3].rotAxisY, joints[3].rotAxisZ), angles[3]), new Point3D(joints[3].rotPointX, joints[3].rotPointY, joints[3].rotPointZ));
            F4.Children.Add(T);
            F4.Children.Add(R);
            F4.Children.Add(F3);

            //as before
            F5 = new Transform3DGroup();
            T = new TranslateTransform3D(0, 0, 0);
            R = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(joints[4].rotAxisX, joints[4].rotAxisY, joints[4].rotAxisZ), angles[4]), new Point3D(joints[4].rotPointX, joints[4].rotPointY, joints[4].rotPointZ));
            F5.Children.Add(T);
            F5.Children.Add(R);
            F5.Children.Add(F4);

            //NB: I was having a nightmare trying to understand why it was always rotating in a weird way... SO I realized that the order in which
            //you add the Children is actually VERY IMPORTANT in fact before I was applyting F and then T and R, but the previous transformation
            //Should always be applied as last (FORWARD Kinematics)
            F6 = new Transform3DGroup();
            T = new TranslateTransform3D(0, 0, 0);
            R = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(joints[5].rotAxisX, joints[5].rotAxisY, joints[5].rotAxisZ), angles[5]), new Point3D(joints[5].rotPointX, joints[5].rotPointY, joints[5].rotPointZ));
            F6.Children.Add(T);
            F6.Children.Add(R);
            F6.Children.Add(F5);

            //Gripper 1
            F7 = new Transform3DGroup();
            T = new TranslateTransform3D(0, 0, 0);
            R = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(joints[6].rotAxisX, joints[6].rotAxisY, joints[6].rotAxisZ), angles[6]), new Point3D(joints[6].rotPointX, joints[6].rotPointY, joints[6].rotPointZ));
            F7.Children.Add(T);
            F7.Children.Add(R);
            F7.Children.Add(F5);

            //Gripper 2
            //F8 = new Transform3DGroup();
            //T = new TranslateTransform3D(0, 0, 0);
            //R = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(joints[7].rotAxisX, joints[7].rotAxisY, joints[7].rotAxisZ), angles[7]), new Point3D(joints[7].rotPointX, joints[7].rotPointY, joints[7].rotPointZ));
            //F8.Children.Add(T);
            //F8.Children.Add(R);
            //F8.Children.Add(F6);



            joints[0].model.Transform = F1; //First joint
            joints[1].model.Transform = F2; //Second joint (the "biceps")
            joints[2].model.Transform = F3; //third joint (the "knee" or "elbow")
            joints[3].model.Transform = F4; //the "forearm"
            joints[4].model.Transform = F5; //the tool plate
            joints[5].model.Transform = F6; //the tool
            joints[6].model.Transform = F7; //the gripper 1
            //joints[7].model.Transform = F8; //the gripper 2

            Tx.Content = joints[4].model.Bounds.Location.X;
            Ty.Content = joints[4].model.Bounds.Location.Y;
            Tz.Content = joints[4].model.Bounds.Location.Z;
            //tx_copy.content = geom.bounds.location.x;
            //ty_copy.content = geom.bounds.location.y;
            //tz_copy.content = geom.bounds.location.z;

            return new Vector3D(joints[4].model.Bounds.Location.X, joints[4].model.Bounds.Location.Y, joints[4].model.Bounds.Location.Z);
        }


    private void jointBtn_Click(object sender, RoutedEventArgs e)
        {

        }

        private void btn_Click(object sender, RoutedEventArgs e)
        {

        }

        private void listBox_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {

        }

        private void pozisyonKaydet(object sender, RoutedEventArgs e)
        {
            try
            {
                //serialPort1.Write("<AYARLA" + "," + textBox_send.Text +","+ textBox_send1.Text + "," + textBox_send2.Text + "," + textBox_send3.Text + "," + textBox_send4.Text+">");
                // textBox'lardan verileri okuyalım
                double x = joint1.Value;
                double y = joint2.Value;
                double z = joint3.Value;
                double w = joint4.Value;
                double t = joint5.Value;
                x = Math.Round(x, 2);
                y = Math.Round(y, 2);
                z = Math.Round(z, 2);
                w = Math.Round(w, 2);
                t = Math.Round(t, 2);


                // verileri bir string olarak birleştirelim
                string data = x + "/" + y + "/" + z + "/" + w + "/" + t;

                // veriyi listBox'a ekleyelim
                gorevList.Items.Add(data);
            }
            catch (Exception error)
            {
                System.Windows.Forms.MessageBox.Show(error.Message);
            }
        }

        private void gorevSil(object sender, RoutedEventArgs e)
        {
            // listBox'tan seçilen öğeyi alalım
            object item = gorevList.SelectedItem;

            // eğer bir öğe seçilmişse
            if (item != null)
            {
                // öğeyi listBox'tan kaldıralım
                gorevList.Items.Remove(item);
            }
            // eğer bir öğe seçilmemişse
            else
            {
                // bir uyarı mesajı gösterelim
                System.Windows.MessageBox.Show("Lütfen bir öğe seçin!");
            }
        }

        private void gorevTemizle(object sender, RoutedEventArgs e)
        {
            gorevList.Items.Clear();
        }

        private void gorevBasla(object sender, RoutedEventArgs e)
        {
            //// ListBox'taki öğe sayısını alıyoruz
            int ogeSayisi = gorevList.Items.Count;

            //// Seri porttan gönderilecek karakter dizisini oluşturuyoruz
            string mesaj = "<GOREV," + ogeSayisi.ToString() + ",";

            // listboxtaki elemanları alıp / ile ayırıp int cinsine çevirip mesaja ekliyoruz
            
            foreach (var item in gorevList.Items)
            {
                string data = item.ToString();
                //data = data.Replace(',', '.');
                string[] degerler = data.Split('/');
                double[] sayilar = new double[degerler.Length]; // int tipinde bir dizi oluştur
                for (int i = 0; i < degerler.Length; i++) // döngü ile tüm elemanları gez
                {
                    sayilar[i] = double.Parse(degerler[i]); // her elemanı int tipine çevir ve yeni dizinin ilgili indisine ata
                }

                mesaj += (int)sayilar[0] + "," + (int)sayilar[1] + "," + (int)sayilar[2] + "," + (int)sayilar[3] + "," + (int)sayilar[4] + ",";
            }
            //// Son karakteri '>' ile değiştiriyoruz
            mesaj += ">";

            //// Seri porttan mesajı gönderiyoruz
            try
            {
                sp.Write(mesaj);
            }
            catch (Exception error)
            {
                System.Windows.MessageBox.Show(error.Message);
            }
            
        }

        private void gorevPGoster(object sender, RoutedEventArgs e)
        {
            // listBox'tan seçilen öğeyi alalım
            object item = gorevList.SelectedItem;

            // eğer bir öğe seçilmişse
            if (item != null)
            {
                string data = item.ToString();
                string[] degerler = data.Split('/');
                joint1.Value = Convert.ToDouble(degerler[0]);
                joint2.Value = Convert.ToDouble(degerler[1]);
                joint3.Value = Convert.ToDouble(degerler[2]);
                joint4.Value = Convert.ToDouble(degerler[3]);
                joint5.Value = Convert.ToDouble(degerler[4]);
                
            }
            // eğer bir öğe seçilmemişse
            else
            {
                // bir uyarı mesajı gösterelim
                System.Windows.MessageBox.Show("Lütfen bir öğe seçin!");
            }
        }

        private void gorevSimulasyon(object sender, RoutedEventArgs e)
        {
            // BackgroundWorker çalışıyor mu diye kontrol et
            if (!isAnimateMission)
            {
                // Çalışmıyorsa, çalıştır
                bgw.RunWorkerAsync();
                btnG4.Content = "Durdur";
                // Butonun rengini #FFC6423A yap
                btnG4.Background = new SolidColorBrush(ColorHelper.HexToColor("#FFC6423A"));
                
            }
            else
            {
                bgw.CancelAsync();
                btnG4.Content = "Görev Önizleme";
                // Butonun rengini #FF008CC8 yap
                btnG4.Background = new SolidColorBrush(ColorHelper.HexToColor("#FF008CC8"));
            }
        }

        private void modelUpdaterStartStop(object sender, RoutedEventArgs e)
        {
            // BackgroundWorker çalışıyor mu diye kontrol et
            if (!isModelUpdater)
            {
                // Çalışmıyorsa, çalıştır
                isModelUpdater = true;
                modelUpdater.RunWorkerAsync();
                btn3.Content = "Durdur";
                // Butonun rengini #FFC6423A yap
                btn3.Background = new SolidColorBrush(ColorHelper.HexToColor("#FFC6423A"));

            }
            else
            {
                isModelUpdater = false;
                //modelUpdater.CancelAsync();
                btn3.Content = "Model Güncelle";
                // Butonun rengini #FF008CC8 yap
                btn3.Background = new SolidColorBrush(ColorHelper.HexToColor("#FF008CC8"));
            }
        }   

        private void gorevKaydet(object sender, RoutedEventArgs e)
        {
            // Verileri kaydetmek için SaveFileDialog kullan
            // SaveFileDialog nesnesi oluştur
            SaveFileDialog saveFile = new SaveFileDialog();
            // Filtreyi txt dosyaları olarak ayarla
            saveFile.Filter = "Text File|*.txt";
            // Pencereyi göster
            if (saveFile.ShowDialog() == System.Windows.Forms.DialogResult.OK)
            {
                // Seçilen dosya adını al
                string fileName = saveFile.FileName;
                // StreamWriter nesnesi oluştur
                StreamWriter writer = new StreamWriter(fileName);
                // listBox'taki tüm verileri yaz
                foreach (var item in gorevList.Items)
                {
                    writer.WriteLine(item.ToString());
                }
                // StreamWriter nesnesini kapat
                writer.Close();
                // Mesaj göster
                //MessageBox.Show("Veriler başarıyla kaydedildi.");
            }
        }

        private void gorevYukle(object sender, RoutedEventArgs e)
        {
            // OpenFileDialog nesnesi oluştur
            OpenFileDialog openFile = new OpenFileDialog();
            // Filtreyi txt dosyaları olarak ayarla
            openFile.Filter = "Text File|*.txt";
            // Pencereyi göster
            if (openFile.ShowDialog() == System.Windows.Forms.DialogResult.OK)
            {
                // Seçilen dosya adını al
                string fileName = openFile.FileName;
                // Dosyadaki tüm satırları oku
                string[] lines = File.ReadAllLines(fileName);
                // listBox'ı temizle
                gorevList.Items.Clear();
                // listBox'a verileri ekle
                foreach (var line in lines)
                {
                    gorevList.Items.Add(line);
                }
                // Mesaj göster
                //MessageBox.Show("Veriler başarıyla yüklendi.");
            }
        }

        private void sifiraGit(object sender, RoutedEventArgs e)
        {
            joint1.Value = 0;
            joint2.Value = 0;
            joint3.Value = 0;
            joint4.Value = 0;
            joint5.Value = 0;
            
            try
            {
                sp.Write("<AYARLA" + "," + "0" + "," + "0" + "," + "0" + "," + "0" + "," + "0" + ">");
                sp.Write("<" + "KONUMAGIT" + ">");

            }
            catch (Exception error)
            {
                System.Windows.MessageBox.Show(error.Message);
            }
        }

        private void pozisyonAyarla(object sender, RoutedEventArgs e)
        {
            // joint değerlerini int değerlere çevir

            sp.Write("<AYARLA" + "," + (int)joint1.Value + "," + (int)joint2.Value + "," + (int)joint3.Value + "," + (int)joint4.Value + "," + (int)joint5.Value + ">");
            //sp.Write("<" + "BILGI2" + ">");
        }

        private void pozisyonaGit(object sender, RoutedEventArgs e)
        {
            try
            {
                sp.Write("<" + "KONUMAGIT" + ">");

            }
            catch (Exception error)
            {
                System.Windows.MessageBox.Show(error.Message);
            }
        }

        private void hareketEt(object sender, RoutedEventArgs e)
        {
            try
            {
                sp.Write("<" + "BASLA" + ">");

            }
            catch (Exception error)
            {
                System.Windows.MessageBox.Show(error.Message);
            }
        }

        private void sifirAyarla(object sender, RoutedEventArgs e)
        {
            try
            {
                sp.Write("<" + "SIFIRLA" + ">");

            }
            catch (Exception error)
            {
                System.Windows.MessageBox.Show(error.Message);
            }
        }

        private void gercekPozGit(object sender, RoutedEventArgs e)
        {
            joint1.Value = joints[0].realAngle;
            joint2.Value = joints[1].realAngle;
            joint3.Value = joints[2].realAngle;
            joint4.Value = joints[3].realAngle;
            joint5.Value = joints[4].realAngle;
            joint6.Value = joints[5].realAngle;
        }
    }
}
