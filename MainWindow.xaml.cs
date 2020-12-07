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
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Diagnostics;
using Microsoft.Kinect;
using Alea;
using Alea.Parallel;
using System.IO;
using Stream = System.IO.Stream;

namespace kinect_test
{
    /// <summary>
    /// MainWindow.xaml の相互作用ロジック
    /// </summary>
    public partial class MainWindow : Window
    {
        /*Sensor→Source→Reader→FrameReference→Frame*/
        private readonly int cbytesPerPixel = 4;
        //本体への参照
        KinectSensor kinect;
        CoordinateMapper mapper;
        //取得するカラー画像の詳細情報
        FrameDescription colorFrameDescription;
        //取得するカラー画像のフォーマット
        ColorImageFormat colorImageFormat;
        //カラー画像を継続的に読み込むためのリーダ
        ColorFrameReader colorFrameReader;

        //取得する深度画像の詳細情報
        FrameDescription depthFrameDescription;
        //深度画像を継続的に読み込むためのリーダ
        DepthFrameReader depthFrameReader;
        //センサーからフレームデータを受け取る中間ストレージ
        private ushort[] depthFrameData = null;
        private byte[] depthPixels = null;
        private WriteableBitmap bitmap = null;

        /// <summary>
        /// Map depth range to byte range
        /// </summary>
        private const int MapDepthToByte = 8000 / 256;

        //Mapperの作成（DepthとColorの位置合わせ）
        BitmapSource colorBitmap;
        BitmapSource depthBitmap;


        public MainWindow()
        {
            InitializeComponent();
            try
            {
                //Kinectへの参照を確保
                this.kinect = KinectSensor.GetDefault();
                this.colorFrameReader = this.kinect.ColorFrameSource.OpenReader();
                this.colorFrameReader.FrameArrived += ColorFrameReader_FrameArrived;
                //読み込む画像のフォーマットとリーダーを設定
                this.colorImageFormat = ColorImageFormat.Bgra;
                this.colorFrameDescription
                    = this.kinect.ColorFrameSource.CreateFrameDescription(this.colorImageFormat);

                //深度について
                this.depthFrameDescription = this.kinect.DepthFrameSource.FrameDescription;
                this.depthFrameReader = this.kinect.DepthFrameSource.OpenReader();
                this.depthFrameReader.FrameArrived += this.Reader_DepthFrameArrived;
                //受信して変換するピクセルを配置するためのスペースを割り当てます
                this.depthFrameData = new ushort[depthFrameDescription.Width * depthFrameDescription.Height];
                this.depthPixels = new byte[depthFrameDescription.Width * depthFrameDescription.Height * this.cbytesPerPixel];
                //create the bitmap to display
                this.bitmap = new WriteableBitmap(depthFrameDescription.Width, depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);
                //動作開始
                kinect.Open();
            }
            catch
            {
                MessageBox.Show("Kinectの検出できやせん");
            }
        }

        ///<summary>
        ///kinectがカラー画像を取得したときに実行されるメソッド
        ///</summary>
        ///<param name="sender">
        ///イベントを通知したオブジェクト（kinect）
        ///</param>
        ///<param name="e">
        ///イベント時に渡されるデータ（カラー画像）
        ///</param>
        void ColorFrameReader_FrameArrived(object sender, ColorFrameArrivedEventArgs e)
        {
            //通知されたフレームの取得
            ColorFrame colorFrame = e.FrameReference.AcquireFrame();

            //例外処理
            if (colorFrame == null)
            {
                return;
            }

            //画像情報を確保するバッファ（領域）を確保
            //高さ*幅*画素あたりのデータ量
            byte[] colors = new byte[this.depthFrameDescription.Width
                * this.depthFrameDescription.Height * this.colorFrameDescription.BytesPerPixel];

            //用意した領域に画素情報を複製
            //colorFrame.CopyConvertedFrameDataToArray(colors, this.colorImageFormat);

            //画素情報をビットマップとして扱う
            BitmapSource bitmapSource = BitmapSource.Create(this.depthFrameDescription.Width,
                this.depthFrameDescription.Height,
                96, 96, PixelFormats.Bgra32, null, colors, this.depthFrameDescription.Width * (int)this.colorFrameDescription.BytesPerPixel);

            //this.Images2.Source = bitmapSource;

            colorFrame.Dispose();
        }

        private void Reader_DepthFrameArrived(object sender, DepthFrameArrivedEventArgs e)
        {
            var sw = new System.Diagnostics.Stopwatch();
            sw.Start();
            ushort minDepth = 0;
            ushort maxDepth = 0;
            //DepthSpacePoint depthPoint = kinect.CoordinateMapper.MapCameraPointToDepthSpace();

            //フレームの取得
            DepthFrame depthFrame = e.FrameReference.AcquireFrame();
            //例外処理
            if (depthFrame == null)
            {
                return;
            }

            depthFrame.CopyFrameDataToArray(this.depthFrameData);
            minDepth = depthFrame.DepthMinReliableDistance;
            maxDepth = depthFrame.DepthMaxReliableDistance;

            //depth座標に対応するcolor座標を取得
            var colorSpace = new ColorSpacePoint[depthFrameDescription.LengthInPixels];//DepthSpacePointは深度画像のピクセル画像.xy
            mapper.MapDepthFrameToColorSpace(depthFrameData, colorSpace);

            for (int i = 0; i < this.depthFrameData.Length; ++i)
            {
                ushort depth = this.depthFrameData[i];
                int image_x = i % 512;
                int image_y = i / 512;

                int world_z = depth;
                int world_x = image_x * world_z;
            }

            int colorPixelIndex = 0;
            for (int i = 0; i < this.depthFrameData.Length; ++i)
            {
                // Get the depth for this pixel
                ushort depth = this.depthFrameData[i];

                // To convert to a byte, we're mapping the depth value to the byte range.
                // Values outside the reliable depth range are mapped to 0 (black).
                byte intensity = (byte)(depth >= minDepth && depth <= maxDepth ? (depth / MapDepthToByte) : 0);

                // Write out blue byte
                this.depthPixels[colorPixelIndex++] = intensity;

                // Write out green byte
                this.depthPixels[colorPixelIndex++] = intensity;

                // Write out red byte                        
                this.depthPixels[colorPixelIndex++] = intensity;

                // Write out alpha byte                        
                this.depthPixels[colorPixelIndex++] = 255;
            }

            
            BitmapSource bitmap_d = BitmapSource.Create(this.depthFrameDescription.Width,
                this.depthFrameDescription.Height,
                96, 96, PixelFormats.Bgra32, null, depthPixels, this.depthFrameDescription.Width * (int)this.colorFrameDescription.BytesPerPixel);

            Images.Source = bitmap_d;

            // BitmapSourceを保存する
            if (sw.Elapsed.Seconds < 10)
            {
                using (Stream stream = new FileStream("test.png", FileMode.Create))
                {
                    PngBitmapEncoder encoder = new PngBitmapEncoder();
                    encoder.Frames.Add(BitmapFrame.Create(bitmap_d));
                    encoder.Save(stream);
                }
            }
        }
        

        /// <summary>
        /// この WPF アプリケーションが終了するときに実行されるメソッド。
        /// </summary>
        /// <param name="e">
        /// イベントの発生時に渡されるデータ。
        /// </param>
        protected override void OnClosed(EventArgs e)
        {
            base.OnClosed(e);
            //カラー画像の取得を中止して、関連するリソースを破棄する。
            if (this.colorFrameReader != null)
            {
                this.colorFrameReader.Dispose();
                this.colorFrameReader = null;
            }

            //Kinect を停止して、関連するリソースを破棄する。
            if (this.kinect != null)
            {
                this.kinect.Close();
                this.kinect = null;
            }
        }
    }
}
