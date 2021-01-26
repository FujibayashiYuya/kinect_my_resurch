using System;
using System.Diagnostics;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Microsoft.Kinect;
using OpenCvSharp;
using OpenCvSharp.WpfExtensions;
using System.IO;
using Stream = System.IO.Stream;

namespace kinect_test
{
    /// <summary>
    /// MainWindow.xaml の相互作用ロジック
    /// </summary>
    public partial class MainWindow : System.Windows.Window
    {
        /*Sensor→Source→Reader→FrameReference→Frame*/
        //本体関係
        KinectSensor kinect;
        CoordinateMapper mapper;
        MultiSourceFrameReader multiReader;

        // Color関係
        FrameDescription colorFrameDescription;
        //取得するカラー画像のフォーマット
        ColorImageFormat colorImageFormat = ColorImageFormat.Bgra;
        private byte[] colorFrameData;

        // Depth関係
        FrameDescription depthFrameDescription;
        //センサーからフレームデータを受け取る中間ストレージ
        private ushort[] depthFrameData = null;

        /// <summary>
        /// Map depth range to byte range
        /// </summary>
        private const int MapDepthToByte = 8000 / 256;

        //内部データ
        private CameraIntrinsics calibrationData;


        bool click;

        //流れ
        //カラー画像・深度画像の取得
        //カラー画像深度画像の平滑化（平滑化してから法線を求める）
        //深度画像を法線情報に変換
        //法線情報とカラー情報をまとめる
        //Kmeans法でカラー情報の分離
        //最小二乗法で法線方向の照度を求める

        public MainWindow()
        {
            InitializeComponent();
            try
            {
                //Kinectへの参照を確保
                this.kinect = KinectSensor.GetDefault();
                this.mapper = kinect.CoordinateMapper;

                //Color情報取得
                this.colorFrameDescription
                    = kinect.ColorFrameSource.CreateFrameDescription(this.colorImageFormat);
                this.colorFrameData = new byte[colorFrameDescription.LengthInPixels *
                                            colorFrameDescription.BytesPerPixel];

                //深度について
                this.depthFrameDescription = this.kinect.DepthFrameSource.FrameDescription;
                //受信して変換するピクセルを配置するためのスペースを割り当てます
                this.depthFrameData = new ushort[depthFrameDescription.Width * depthFrameDescription.Height];

                this.multiReader = this.kinect.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth);
                this.multiReader.MultiSourceFrameArrived += multiReader_MultiSourceFrameArrived;
                kinect.Open();
            }
            catch
            {
                MessageBox.Show("Kinectの検出できやせん");
            }
        }

        void multiReader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            var multiFrame = e.FrameReference.AcquireFrame();
            //例外処理
            if (multiFrame == null)
            {
                MessageBox.Show("マルチフレームがありません");
                return;
            }

            //データの取得
            var colorFrame = multiFrame.ColorFrameReference.AcquireFrame();
            var depthFrame = multiFrame.DepthFrameReference.AcquireFrame();
            if (colorFrame == null || depthFrame == null)
            {
                return;
            }
            colorFrame.CopyConvertedFrameDataToArray(colorFrameData, this.colorImageFormat);
            depthFrame.CopyFrameDataToArray(this.depthFrameData);

            //描写
            //Depthのサイズで作成
            var colorImageBuffer = new byte[depthFrameDescription.LengthInPixels * colorFrameDescription.BytesPerPixel];
            var depthImageBuffer = new byte[depthFrameDescription.LengthInPixels * colorFrameDescription.BytesPerPixel];
            //Depth情報保存用(y,x)
            UInt16[,] depthBuffer = new UInt16[424, 512];
            //Depth座標系に対応するカラー座標系の取得
            var colorSpace = new ColorSpacePoint[depthFrameDescription.LengthInPixels];
            mapper.MapDepthFrameToColorSpace(depthFrameData, colorSpace);
            for (int i = 0; i < this.depthFrameData.Length; ++i)
            {
                int colorX = (int)colorSpace[i].X;
                int colorY = (int)colorSpace[i].Y;
                if ((colorX < 0) || (colorFrameDescription.Width <= colorX) ||
                             (colorY < 0) || (colorFrameDescription.Height <= colorY))
                {
                    continue;
                }

                //カラー画像のインデックス
                int colorIndex = colorY * colorFrameDescription.Width + colorX;
                int colorImageIndex = (int)(i * colorFrameDescription.BytesPerPixel);
                int colorBufferIndex = (int)(colorIndex * colorFrameDescription.BytesPerPixel);

                colorImageBuffer[colorImageIndex + 0] = colorFrameData[colorBufferIndex + 0];
                colorImageBuffer[colorImageIndex + 1] = colorFrameData[colorBufferIndex + 1];
                colorImageBuffer[colorImageIndex + 2] = colorFrameData[colorBufferIndex + 2];

                //深度画像
                byte intensity = (byte)(depthFrameData[i] % 255);
                depthImageBuffer[colorImageIndex++] = intensity;
                depthImageBuffer[colorImageIndex++] = intensity;
                depthImageBuffer[colorImageIndex++] = intensity;
            }
            BitmapSource collor = BitmapSource.Create(this.depthFrameDescription.Width,
                this.depthFrameDescription.Height,
                96, 96, PixelFormats.Bgr32, null, colorImageBuffer, this.depthFrameDescription.Width * (int)this.colorFrameDescription.BytesPerPixel);

            BitmapSource depth = BitmapSource.Create(this.depthFrameDescription.Width,
                this.depthFrameDescription.Height,
                96, 96, PixelFormats.Bgr32, null, depthImageBuffer, this.depthFrameDescription.Width * (int)this.colorFrameDescription.BytesPerPixel);

            Images.Source = collor;
            Images2.Source = depth;

            if (click == true)
            {
                //カラー情報とデプス情報を別で保存して、メモリ解放する
                click = false;
                calibrationData = mapper.GetDepthCameraIntrinsics();
                MessageBox.Show("test");
                /*-------------Depthだけを保存せず、いきなり頂点を求める----------------------
                //Depth情報を保存
                for (int i = 0; i < this.depthFrameData.Length; ++i)
                {
                    int x = i % 512;
                    int y = i / 512;
                    depthBuffer[y, x] = depthFrameData[i];
                }
                ------------------------------------------------------------------------------*/
                    // BitmapSourceを保存する
                    /*
                    using (Stream stream = new FileStream("test.png", FileMode.Create))
                    {
                        PngBitmapEncoder encoder = new PngBitmapEncoder();
                        encoder.Frames.Add(BitmapFrame.Create(depth));
                        encoder.Save(stream);
                    }*/
                    Mat src = BitmapSourceConverter.ToMat(depth);
                
                //頂点マップの作成
                var depthData = new int[depthFrameDescription.LengthInPixels * colorFrameDescription.BytesPerPixel];
                for (int i = 0; i < this.depthFrameData.Length; ++i)
                {
                    int colorImageIndex = (int)(i * colorFrameDescription.BytesPerPixel);
                    //テクスチャ座標
                    var im_x = i % depthFrameDescription.Width;
                    var im_y = i / depthFrameDescription.Width;

                    //uv座標
                    var u = im_x - depthFrameDescription.Width * 0.5;
                    var v = depthFrameDescription.Height * 0.5 - im_y;

                    //正規化用の座標最大値・最小値
                    var range_x = depthFrameDescription.Width / calibrationData.FocalLengthX * 4500;
                    var range_y = depthFrameDescription.Height / calibrationData.FocalLengthY * 4500;

                    //頂点座標
                    depthData[colorImageIndex++] = (int)((u - calibrationData.PrincipalPointX) / calibrationData.FocalLengthX * depthFrameData[i]);//x
                    depthData[colorImageIndex++] = (int)((v - calibrationData.PrincipalPointY) / calibrationData.FocalLengthY * depthFrameData[i]);//y
                    depthData[colorImageIndex++] = (int)depthFrameData[i];//z
                }

                //法線マップの作成
                var normalData = new float[depthFrameDescription.LengthInPixels * colorFrameDescription.BytesPerPixel];
                for (int i = 0; i < this.depthFrameData.Length; ++i)
                {
                    int normalImageIndex = (int)(i * colorFrameDescription.BytesPerPixel);
                    //外積はベクトルで計算したほうが楽（書き直し）
                    normalData[normalImageIndex + 0] = depthData[normalImageIndex + 3] - depthData[normalImageIndex - 3];
                    normalData[normalImageIndex + 1] = depthData[normalImageIndex + depthFrameDescription.Width * 3] - depthData[normalImageIndex - depthFrameDescription.Width * 3];
                    normalData[normalImageIndex + 2] = 
                }


                }
            colorFrame.Dispose();
            depthFrame.Dispose();
        }

        //画像の平滑化
        private void ImageProgress()
        {
            Mat img;
        }

        void OnClick(object sender, RoutedEventArgs e)
        {
            click = true;
        }

        const int CLASS_NUM = 16;
        private void Kmeans_segmentation()
        {
            //Cv2.Kmeans;
        }

        /// <summary>
        /// この WPF アプリケーションが終了するときに実行されるメソッド。
        /// </summary>
        /// <param name="e">
        /// イベントの発生時に渡されるデータ。
        /// </param>
        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (multiReader != null)
            {
                multiReader.MultiSourceFrameArrived -= multiReader_MultiSourceFrameArrived;
                multiReader.Dispose();
                multiReader = null;
            }

            if (kinect != null)
            {
                kinect.Close();
                kinect = null;
            }
        }
    }
}
