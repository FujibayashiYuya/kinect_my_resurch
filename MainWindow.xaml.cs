using System;
using System.Diagnostics;
using System.Threading.Tasks;
using System.Windows;
using System.Numerics;
using System.Runtime;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Collections.Generic;
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

            //クリック時
            if (click == true)
            {
                //カラー情報とデプス情報を別で保存して、メモリ解放する
                click = false;
                calibrationData = mapper.GetDepthCameraIntrinsics();
                // BitmapSourceを保存する
                /*
                using (Stream stream = new FileStream("test.png", FileMode.Create))
                {
                    PngBitmapEncoder encoder = new PngBitmapEncoder();
                    encoder.Frames.Add(BitmapFrame.Create(depth));
                    encoder.Save(stream);
                }*/
                //Mat src = BitmapSourceConverter.ToMat(depth);

                //頂点マップの作成
                var vertexData = new int[depthFrameDescription.LengthInPixels * colorFrameDescription.BytesPerPixel];
                double[,] normalData = new double[depthFrameDescription.LengthInPixels, 3];
                double[] hsi = new double[depthFrameDescription.LengthInPixels * colorFrameDescription.BytesPerPixel];
                depthFrameData = BilateralFilter(depthFrameData);
                depthFrameData = BilateralFilter(depthFrameData);
                vertexData = VertexmapCreate(depthFrameData);

                //法線マップの作成
                normalData = NormalmapCreate(vertexData);

                hsi = Create_Hsi(colorImageBuffer, hsi);

                //カラー画像のクラスタリング
                //Kmeans_segmentation(colorImageBuffer, vertexData);
                Km_hsi(hsi, normalData);
            }
            colorFrame.Dispose();
            depthFrame.Dispose();
        }

        private void DepthInterpolation(ushort[] depthFrameData)
        {
            for (int i = 0; i < depthFrameData.Length; i++)
            {

            }
        }


        /*==========================深度情報から頂点マップを作成する関数=======================================*/
        private int[] VertexmapCreate(ushort[] DepthData)
        {
            //頂点データ
            var vertexData = new int[depthFrameDescription.LengthInPixels * colorFrameDescription.BytesPerPixel];
            for (int i = 0; i < DepthData.Length; ++i)
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
                vertexData[colorImageIndex++] = (int)((u - calibrationData.PrincipalPointX) / calibrationData.FocalLengthX * DepthData[i]);//x
                vertexData[colorImageIndex++] = (int)((v - calibrationData.PrincipalPointY) / calibrationData.FocalLengthY * DepthData[i]);//y
                vertexData[colorImageIndex++] = (int)DepthData[i];//z
            }

            //頂点マップを画像用にRGBの範囲で正規化する。
            var vertexImage = new byte[depthFrameDescription.LengthInPixels * colorFrameDescription.BytesPerPixel];
            for (int i = 0; i < DepthData.Length; ++i)
            {
                int colorImageIndex = (int)(i * colorFrameDescription.BytesPerPixel);
                //正規化用の座標最大値・最小値
                var range_x = depthFrameDescription.Width / calibrationData.FocalLengthX * 4500;
                var range_y = depthFrameDescription.Height / calibrationData.FocalLengthY * 4500;

                //頂点座標
                vertexImage[colorImageIndex] = (byte)(vertexData[colorImageIndex] / range_x * 255);//x
                vertexImage[colorImageIndex + 1] = (byte)(vertexData[colorImageIndex + 1] / range_y * 255);//y
                vertexImage[colorImageIndex + 2] = (byte)(255 * (vertexData[colorImageIndex + 2] - 500) / 7500);//z
            }
            //頂点マップの表示
            BitmapSource vertexMap = BitmapSource.Create(this.depthFrameDescription.Width,
                this.depthFrameDescription.Height,
                96, 96, PixelFormats.Bgr32, null, vertexImage, this.depthFrameDescription.Width * (int)this.colorFrameDescription.BytesPerPixel);
            Mat src = BitmapSourceConverter.ToMat(vertexMap);
            Cv2.ImShow("Test", src);

            return vertexData;
        }

        /*=========================================頂点マップから法線マップを作成する関数(Holzerらの手法)====================================================================*/
        private double[,] NormalmapCreate(int[] VertexData)
        {
            //x方向:vx , y方向:vy , 法線方向:normalData
            double[,] normalData = new double[depthFrameDescription.LengthInPixels, 3];
            double[] vx = new double[3];
            double[] vy = new double[3];
            double[] n = new double[] { 0, 0, 0 };
            int px0, px1, py0, py1;
            int y_dis = (int)(depthFrameDescription.Width * colorFrameDescription.BytesPerPixel);
            int x_dis = (int)(colorFrameDescription.BytesPerPixel);
            for (int i = 0; i < this.depthFrameData.Length; ++i)
            {
                int pIndex = (int)(i * colorFrameDescription.BytesPerPixel);
                //x方向の傾きベクトル
                px0 = pIndex - x_dis < 0 ? 0 : pIndex - x_dis;
                px1 = pIndex + x_dis > VertexData.Length - 1 ? 0 : pIndex + x_dis;
                vx[0] = (VertexData[px1] - VertexData[px0]) * 0.5;
                vx[1] = (VertexData[++px1] - VertexData[++px0]) * 0.5;
                vx[2] = (VertexData[++px1] - VertexData[++px0]) * 0.5;

                //y方向の傾きベクトル
                py0 = (pIndex - y_dis < 0 ? 0 : pIndex - y_dis);
                py1 = (pIndex + y_dis > VertexData.Length - 1 ? 0 : pIndex + y_dis);
                vy[0] = (VertexData[py1] - VertexData[py0]) * 0.5;
                vy[1] = (VertexData[++py1] - VertexData[++py0]) * 0.5;
                vy[2] = (VertexData[++py1] - VertexData[++py0]) * 0.5;
                //↑でおかしい？
                if (vx[0] != 0 || vx[1] != 0 || vx[2] != 0)//まとめるとNoNがでてきた
                {
                    if (vy[0] != 0 || vy[1] != 0 || vy[2] != 0)
                    {
                        n = VecNormalized(VecCross(vx, vy));
                    }
                }
                normalData[i, 0] = n[0];
                normalData[i, 1] = n[1];
                normalData[i, 2] = n[2];
                //Debug.WriteLine(n[0] + n[1] + n[2] + " " + vx[0] + vx[1] + vx[2] + " " + vy[0]+vy[1]+vy[2]);
            }

            //画像に出力
            var normalImage = new byte[depthFrameDescription.LengthInPixels * colorFrameDescription.BytesPerPixel];
            for (int i = 0; i < depthFrameData.Length; ++i)
            {
                int normalImageIndex = (int)(i * colorFrameDescription.BytesPerPixel);
                //頂点座標
                normalImage[normalImageIndex + 0] = (byte)((normalData[i, 0] + 1) * 127.5);//x
                normalImage[normalImageIndex + 1] = (byte)((normalData[i, 1] + 1) * 127.5);//y
                normalImage[normalImageIndex + 2] = (byte)(normalData[i, 2] * 255);//z
            }
            //頂点マップの表示
            BitmapSource vertexMap = BitmapSource.Create(this.depthFrameDescription.Width,
                this.depthFrameDescription.Height,
                96, 96, PixelFormats.Bgr32, null, normalImage, this.depthFrameDescription.Width * (int)this.colorFrameDescription.BytesPerPixel);
            Mat src = BitmapSourceConverter.ToMat(vertexMap);
            Cv2.ImShow("normal", src);

            return normalData;
        }

        //配列用テスト(頂点差からとったベクトルを方向ベクトルに正規化)OK
        private double[] VecNormalized(double[] v)
        {
            var dis = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
            double leng = 1 / Math.Sqrt(dis);//doubleの範囲を超えてる可能性
            var n = new double[3];
            n[0] = v[0] * leng;
            n[1] = v[1] * leng;
            n[2] = v[2] * leng;
            return n;
        }

        //外積を求めるOK
        private double[] VecCross(double[] vx, double[] vy)
        {
            var n = new double[3];
            n[0] = vx[1] * vy[2] - vx[2] * vy[1];
            n[1] = vx[2] * vy[0] - vx[0] * vy[2];
            n[2] = vx[0] * vy[1] - vx[1] * vy[0];
            return n;
        }

        //バイラテラルフィルタ
        private ushort[] BilateralFilter(ushort[] depthFrameData)
        {
            int uw = 0;
            int uh = 0;
            double a = -0.001;
            double b = -0.01;
            ushort[] smoothingdepth = new ushort[depthFrameData.Length];
            float w_deno;
            float d_mole;
            for (int i = 0; i < depthFrameData.Length; i++)
            {
                uw = i % depthFrameDescription.Width;
                uh = i / depthFrameDescription.Width;
                w_deno = 0;
                d_mole = 0;
                //5*5のフィルタ
                for (int m = -2; m < 3; m++)//h
                {
                    for (int n = -2; n < 3; n++)//w
                    {
                        if (uh == (i - 2) / depthFrameDescription.Width && uh == (i + 2) / depthFrameDescription.Width
                            && uw - 2 > 0 && uw + 2 < depthFrameDescription.Width && uh + 2 < depthFrameDescription.Height && uh - 2 > 0)
                        {
                            float dis = m * m + n * n;
                            double dep = Math.Pow(depthFrameData[i] - depthFrameData[i + m * depthFrameDescription.Width + n], 2);
                            d_mole += (float)(depthFrameData[i + m * depthFrameDescription.Width + n] * Math.Exp(dis * a) * Math.Exp(dep * b));
                            w_deno += (float)(Math.Exp(dis * a) * Math.Exp(dep * b));
                        }
                    }
                }
                smoothingdepth[i] = (ushort)(d_mole / w_deno);
            }
            return smoothingdepth;
        }


        void OnClick(object sender, RoutedEventArgs e)
        {
            click = true;
        }

        //Kmeans法によるクラスタリング（色＋頂点座標に改良予定）
        private void Kmeans_segmentation(byte[] colorbuffer, int[] vertexdata)
        {
            //Cv2.Kmeans;
            const int CLASS = 8;
            //色(もともとByte)＋頂点座標(int型)の6chのMat(★float型じゃないとダメ)
            Mat test = new Mat(depthFrameDescription.Width * depthFrameDescription.Height, 1, MatType.CV_32SC(6));
            using (Mat src = new Mat(depthFrameDescription.Width * depthFrameDescription.Height, 1, MatType.CV_32FC3))
            {
                using (Mat cluster = new Mat())
                {
                    using (Mat centers = new Mat(CLASS, 1, MatType.CV_32FC3))
                    {
                        int i = 0;
                        long index = 0;
                        //引数byte[]をKmeans()に適した形にする
                        for (int y = 0; y < depthFrameDescription.Height; y++)
                        {
                            for (int x = 0; x < depthFrameDescription.Width; x++, i++)
                            {
                                index = (y * depthFrameDescription.Width + x) * colorFrameDescription.BytesPerPixel;
                                Vec3f vec3f = new Vec3f
                                {
                                    Item0 = colorbuffer[index + 0],
                                    Item1 = colorbuffer[index + 1],
                                    Item2 = colorbuffer[index + 2]
                                };
                                src.Set<Vec3f>(i, vec3f);
                            }
                        }
                        var criteria = new TermCriteria(type: CriteriaType.Eps | CriteriaType.MaxIter, maxCount: 10, epsilon: 1.0);
                        Cv2.Kmeans(src, CLASS, cluster, criteria, 3, KMeansFlags.PpCenters, centers);
                        //for (int g = 0; g < CLASS; g++) Debug.WriteLine(centers.At<Vec3f>(g));
                        i = 0;
                        Mat output = new Mat(depthFrameDescription.Height, depthFrameDescription.Width, MatType.CV_8UC3);
                        for (int y = 0; y < depthFrameDescription.Height; y++)
                        {
                            for (int x = 0; x < depthFrameDescription.Width; x++, i++)
                            {
                                int ind = cluster.Get<int>(i);

                                Vec3b col = new Vec3b();

                                int firstComponent = Convert.ToInt32(Math.Round(centers.At<Vec3f>(ind)[0]));
                                firstComponent = firstComponent > 255 ? 255 : firstComponent < 0 ? 0 : firstComponent;
                                col[0] = Convert.ToByte(firstComponent);

                                int secondComponent = Convert.ToInt32(Math.Round(centers.At<Vec3f>(ind)[1]));
                                secondComponent = secondComponent > 255 ? 255 : secondComponent < 0 ? 0 : secondComponent;
                                col[1] = Convert.ToByte(secondComponent);

                                int thirdComponent = Convert.ToInt32(Math.Round(centers.At<Vec3f>(ind)[2]));
                                thirdComponent = thirdComponent > 255 ? 255 : thirdComponent < 0 ? 0 : thirdComponent;
                                col[2] = Convert.ToByte(thirdComponent);
                                //if (i < 20) Debug.WriteLine(col);
                                output.Set<Vec3b>(y, x, col);
                            }
                        }
                        Cv2.ImShow("km", output);
                    }
                }
            }
        }
        //https://stackoverflow.com/questions/58221925/acces-to-centroid-cluster-color-after-k-means-in-c-sharp
        /*
        src = Cv2.ImDecode(colorbuffer, ImreadModes.Color);
        Debug.WriteLine(src.Data);
        Debug.WriteLine("横幅は" + src.Width);
        src.ConvertTo(src, MatType.CV_32F);

        //Cv2.ImShow("out", src);
        InputArray srcArr = InputArray.Create(src);
        TermCriteria criteria = new TermCriteria(CriteriaType.Eps, 10, 1.0);
        Cv2.Kmeans(src, CLASS, InputOutputArray.Create(cluster), criteria, 1, KMeansFlags.UseInitialLabels, OutputArray.Create(center));
        */
        //byte[]をMatに変換：https://github.com/shimat/opencvsharp/issues/173
        //https://stackoverflow.com/questions/58221925/acces-to-centroid-cluster-color-after-k-means-in-c-sharp
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


        //一時的な可能性
        #region hsi_km
        private double[] Create_Hsi(byte[] colorbuffer, double[] hsi)
        {
            //要素（Ix、Iy、Iz）を求める
            //Ix:-255～255 , Iy:-255*√3 * 0.5～255*√3 * 0.5 , Iz:0～85
            double[] ibuffer = new double[depthFrameDescription.LengthInPixels * colorFrameDescription.BytesPerPixel];
            uint index = 0;
            double th = 0.3333333;
            for (uint i = 0; i < depthFrameData.Length; i++)
            {
                index = i * colorFrameDescription.BytesPerPixel;
                ibuffer[index] = colorbuffer[index] - 0.5 * colorbuffer[index + 1] - 0.5 * colorbuffer[index + 2];   //Ix
                ibuffer[index + 1] = (colorbuffer[index + 1] - colorbuffer[index + 2]) * 0.5 * Math.Sqrt(3);             //Iy
                ibuffer[index + 2] = colorbuffer[index] * th + colorbuffer[index + 1] * th + colorbuffer[index + 2] * th;//Iz   
            }

            //hue saturation intensity を求める
            // hue...0～360    saturation...0～403(大体) intensity...0～85（目安）
            index = 0;
            for (uint j = 0; j < depthFrameData.Length; j++)
            {
                index = j * colorFrameDescription.BytesPerPixel;
                if (ibuffer[index] != 0)
                {
                    hsi[index] = Math.Atan(ibuffer[index + 1] / ibuffer[index]);//hue
                    hsi[index + 1] = Math.Sqrt(ibuffer[index] * ibuffer[index] + ibuffer[index + 1] * ibuffer[index + 1]);//saturation
                    hsi[index + 2] = ibuffer[index + 2];//intensity
                }
                //Debug.WriteLine(hsi[index] + " " + ibuffer[index]);
            }

            return hsi;
        }

        //Kmeans法によるクラスタリング（色＋頂点座標に改良予定）
        private void Km_hsi(double[] hsi, double[,] normaldata)
        {
            //Cv2.Kmeans;
            const int CLASS = 8;
            //色(もともとByte)＋頂点座標(int型)の6chのMat(★float型じゃないとダメ)
            using (Mat src = new Mat(depthFrameDescription.Width * depthFrameDescription.Height, 1, MatType.CV_32FC(4)))
            {
                using (Mat cluster = new Mat())
                {
                    using (Mat centers = new Mat(CLASS, 1, MatType.CV_32FC(4)))
                    {
                        int i = 0;
                        long index = 0;
                        //引数byte[]をKmeans()に適した形にする
                        for (int y = 0; y < depthFrameDescription.Height; y++)
                        {
                            for (int x = 0; x < depthFrameDescription.Width; x++, i++)
                            {
                                index = y * depthFrameDescription.Width + x;
                                if (hsi[i * colorFrameDescription.BytesPerPixel] != 0)
                                {
                                    Vec4f vec4f = new Vec4f
                                    {
                                        Item0 = (float)(hsi[i * colorFrameDescription.BytesPerPixel] * (180 / Math.PI)) + 90,//色相
                                        Item1 = (float)(normaldata[index, 0] + 2) * 2,//法線
                                        Item2 = (float)(normaldata[index, 1] + 2) * 2,
                                        Item3 = (float)(normaldata[index, 2] + 1) * 2
                                    };
                                    src.Set<Vec4f>(i, vec4f);
                                    //Debug.WriteLine(vec4f);
                                }
                            }
                        }

                        var criteria = new TermCriteria(type: CriteriaType.Eps | CriteriaType.MaxIter, maxCount: 10, epsilon: 1.0);
                        Cv2.Kmeans(src, CLASS, cluster, criteria, 3, KMeansFlags.PpCenters, centers);
                        for (int g = 0; g < CLASS; g++) Debug.WriteLine(centers.At<Vec4f>(g));
                        i = 0;
                        byte gs = 255 / CLASS;
                        Mat output = new Mat(depthFrameDescription.Height, depthFrameDescription.Width, MatType.CV_8UC3);
                        for (int y = 0; y < depthFrameDescription.Height; y++)
                        {
                            for (int x = 0; x < depthFrameDescription.Width; x++, i++)
                            {
                                int ind = cluster.Get<int>(i);

                                Vec3b col = new Vec3b();
                                col[0] = (byte)(gs * ind);
                                col[1] = (byte)(gs * ind);
                                col[2] = (byte)(gs * ind);
                                /*
                                int firstComponent = Convert.ToInt32(Math.Round(centers.At<Vec3f>(ind)[0]));
                                firstComponent = firstComponent > 255 ? 255 : firstComponent < 0 ? 0 : firstComponent;
                                col[0] = Convert.ToByte(firstComponent);

                                int secondComponent = Convert.ToInt32(Math.Round(centers.At<Vec3f>(ind)[1]));
                                secondComponent = secondComponent > 255 ? 255 : secondComponent < 0 ? 0 : secondComponent;
                                col[1] = Convert.ToByte(secondComponent);

                                int thirdComponent = Convert.ToInt32(Math.Round(centers.At<Vec3f>(ind)[2]));
                                thirdComponent = thirdComponent > 255 ? 255 : thirdComponent < 0 ? 0 : thirdComponent;
                                col[2] = Convert.ToByte(thirdComponent);
                                //if (i < 20) Debug.WriteLine(col);
                                */
                                output.Set<Vec3b>(y, x, col);
                            }
                        }
                        Cv2.ImShow("km", output);
                        Debug.WriteLine(colorFrameDescription.BytesPerPixel);
                    }
                }
            }
        }
        #endregion hsi_km
        //改良中
        private void Remove_highlight(byte[] colorbuffer)
        {
            //RGB　⇒　Ix, Iy, Iz
            double[] ibuffer = new double[depthFrameDescription.LengthInPixels * colorFrameDescription.BytesPerPixel];
            uint index = 0;
            for (uint i = 0; i < depthFrameData.Length; i++)
            {
                index = i * colorFrameDescription.BytesPerPixel;
                ibuffer[index] = colorbuffer[index] - 0.5 * colorbuffer[index + 1] - 0.5 * colorbuffer[index + 2];   //Ix
                ibuffer[index + 1] = (colorbuffer[index + 1] - colorbuffer[index + 2]) * 0.5 * Math.Sqrt(3);             //Iy
                ibuffer[index + 2] = (double)colorbuffer[index] / 3 + (double)colorbuffer[index + 1] / 3 + (double)colorbuffer[index + 2] / 3;//Iz   
            }

            //Ix, Iy, Iz ⇒　HSI
            double[] hsi = new double[depthFrameDescription.LengthInPixels * colorFrameDescription.BytesPerPixel];
            for (uint j = 0; j < depthFrameData.Length; j++)
            {
                index = j * colorFrameDescription.BytesPerPixel;
                hsi[index] = (int)(Math.Atan(ibuffer[index + 1] / ibuffer[index]) * (180 / Math.PI)) + 90;
                hsi[index + 1] = Math.Sqrt(ibuffer[index] * ibuffer[index] + ibuffer[index + 1] * ibuffer[index + 1]);
                hsi[index + 2] = ibuffer[index + 2];
            }
        }

        //保存用
        private void test()
        {
            //HSI色空間を求める　hue：番号｛saturation、intensity｝
            var hsi = new List<List<Vec2d>>();
            var hsin = new Vec2d[180][];
            //hsiの容量を確保
            hsi.Add(new List<Vec2d>());
            hsi[0].Add(new Vec2d(0, 0));
            hsi[0].Add(new Vec2d(1, 1));
            hsi[1].Add(new Vec2d(1, 1));
            Debug.WriteLine("a" + hsi[0][0] + hsi[0][1]);
            for (int i = 0; i < 181; i++)
            {
                hsi[i].Add(new Vec2d(0, 0));
            }
            double hue = 0;
            int hue_angle = 0;
            double saturation = 0;
            double intensity = 0;
            /*
            for (uint j = 0; j < depthFrameData.Length; j++)
            {
                index = j * colorFrameDescription.BytesPerPixel;
                if (ibuffer[index] != 0)//0の場合、分母が０になり数字がおかしくなる
                {
                    hue = (int)Math.Atan(ibuffer[index + 1] / ibuffer[index]);
                    hue_angle = (int)(hue * (180 / Math.PI)) + 90;
                    saturation = Math.Sqrt(ibuffer[index] * ibuffer[index] + ibuffer[index + 1] * ibuffer[index + 1]);
                    intensity = ibuffer[index + 2];
                    Debug.WriteLine(hue +" , "  + hue_angle + " , "+ saturation +" , " +  intensity);
                    hsi[hue_angle].Add(new Vec2d(saturation, intensity));
                }
            }*/

        }

        /*=============使わなくなった関数=====================================================================================================*/
        private Vector3[] NmC(int[] VertexData)
        {
            var normalData = new int[depthFrameDescription.LengthInPixels * colorFrameDescription.BytesPerPixel];
            var norvecData = new Vector3[depthFrameDescription.LengthInPixels];
            //vx=V(x+1,y)−V(x−1,y) 
            //vy = V(x, y + 1)−V(x, y−1)
            //n(u) = norm(vx×vy)
            var vx = new int[depthFrameDescription.LengthInPixels * colorFrameDescription.BytesPerPixel];
            var vy = new int[depthFrameDescription.LengthInPixels * colorFrameDescription.BytesPerPixel];

            for (int i = 0; i < this.depthFrameData.Length; ++i)
            {
                int vecIndex = (int)(i * colorFrameDescription.BytesPerPixel);
                int x_vec = (int)(colorFrameDescription.BytesPerPixel);
                int y_vec = (int)(depthFrameDescription.Width * colorFrameDescription.BytesPerPixel);

                //とりあえず四隅の値は無しでやる
                if (vecIndex + x_vec > VertexData.Length || vecIndex - x_vec < 0 ||
                    (vecIndex + x_vec) / y_vec != (vecIndex - x_vec) / y_vec ||
                    (vecIndex + y_vec) > VertexData.Length || (vecIndex - y_vec) < 0)
                {
                    //vxについて
                    vx[vecIndex] = 0;
                    vx[vecIndex + 1] = 0;
                    vx[vecIndex + 2] = 0;

                    //vyについて
                    vy[vecIndex] = 0;
                    vy[vecIndex + 1] = 0;
                    vy[vecIndex + 2] = 0;
                }
                else
                {
                    //vxについて
                    vx[vecIndex] = VertexData[vecIndex + x_vec] - VertexData[vecIndex - x_vec];
                    vx[++vecIndex] = VertexData[vecIndex + x_vec] - VertexData[vecIndex - x_vec];
                    vx[++vecIndex] = VertexData[vecIndex + x_vec] - VertexData[vecIndex - x_vec];

                    //vyについて
                    vy[vecIndex] = VertexData[vecIndex + y_vec] - VertexData[vecIndex - y_vec];
                    vy[++vecIndex] = VertexData[vecIndex + y_vec] - VertexData[vecIndex - y_vec];
                    vy[++vecIndex] = VertexData[vecIndex + y_vec] - VertexData[vecIndex - y_vec];
                }
            }
            //外積の方向に注意・法線ベクトルが真逆向かないように(vxからvyにねじを回して進む方向が法線方向になる)
            //法線方向を求める
            for (int j = 0; j < this.depthFrameData.Length; ++j)
            {
                int vecIndex = (int)(j * colorFrameDescription.BytesPerPixel);
                normalData[vecIndex + 0] = vx[vecIndex + 1] * vy[vecIndex + 2] - vx[vecIndex + 2] * vy[vecIndex + 1];
                normalData[vecIndex + 1] = vx[vecIndex + 2] * vy[vecIndex + 0] - vx[vecIndex + 0] * vy[vecIndex + 2];
                normalData[vecIndex + 2] = vx[vecIndex + 0] * vy[vecIndex + 1] - vx[vecIndex + 1] * vy[vecIndex + 0];
                //norvecData[j] = VecNormalized(normalData[vecIndex + 0], normalData[vecIndex + 1], normalData[vecIndex + 2]);
                //if (norvecData[j].X > 2)MessageBox.Show("2こえ");//おかしい2すら超えてる
            }

            //画像出力用
            var normalmap = new byte[depthFrameDescription.LengthInPixels * colorFrameDescription.BytesPerPixel];
            for (int j = 0; j < this.depthFrameData.Length; ++j)
            {
                int vecIndex = (int)(j * colorFrameDescription.BytesPerPixel);
                normalmap[vecIndex + 0] = (byte)((norvecData[j].X + 1) * 127.5);
                normalmap[vecIndex + 1] = (byte)((norvecData[j].Y + 1) * 127.5);
                normalmap[vecIndex + 2] = (byte)(norvecData[j].Z * 255);
            }
            //MessageBox.Show("test");
            BitmapSource normalMap = BitmapSource.Create(this.depthFrameDescription.Width,
                this.depthFrameDescription.Height,
                96, 96, PixelFormats.Bgr32, null, normalmap, this.depthFrameDescription.Width * (int)this.colorFrameDescription.BytesPerPixel);
            Mat src = BitmapSourceConverter.ToMat(normalMap);
            Cv2.ImShow("normal", src);
            return norvecData;
        }

        //積分画像作成
        private uint[] MakeIntegralImage(ushort[] depthFrameData)
        {
            uint[] integralimg = new uint[depthFrameDescription.Width * depthFrameDescription.Height];
            integralimg[0] = depthFrameData[0];
            int xm = 0;
            int ym = 0;
            for (int i = 1; i < depthFrameDescription.Width * depthFrameDescription.Height; i++)
            {
                xm = i % depthFrameDescription.Width;
                ym = i / depthFrameDescription.Width;
                if (ym == 0)
                {
                    integralimg[xm] = integralimg[xm - 1] + depthFrameData[xm];
                }
                else if (xm == 0)
                {
                    integralimg[i] = integralimg[i - depthFrameDescription.Width] + depthFrameData[i];
                }
                else
                {
                    integralimg[i] = integralimg[i - 1] + integralimg[i - depthFrameDescription.Width] - integralimg[i - depthFrameDescription.Width - 1] + depthFrameData[i];
                }
            }
            return integralimg;
        }

        private ushort Depthregionave(uint[] inteim, int m, int n, int r)
        {
            ushort s = (ushort)(inteim[(n + r) * depthFrameDescription.Width + m + r] - inteim[(n + r) * depthFrameDescription.Width - m - r]
                 + inteim[(n - r) * depthFrameDescription.Width + m - r] - inteim[(n - r) * depthFrameDescription.Width - m - r]);
            return s;
        }
    }
}
