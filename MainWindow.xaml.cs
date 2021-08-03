using Microsoft.Kinect;
using OpenCvSharp;
using OpenCvSharp.WpfExtensions;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;

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
                if (depthFrameData[i] < 2000)
                {
                    colorImageBuffer[colorImageIndex + 0] = colorFrameData[colorBufferIndex + 0];//B
                    colorImageBuffer[colorImageIndex + 1] = colorFrameData[colorBufferIndex + 1];//G
                    colorImageBuffer[colorImageIndex + 2] = colorFrameData[colorBufferIndex + 2];//R

                    //深度画像
                    byte intensity = (byte)(depthFrameData[i] % 255);
                    depthImageBuffer[colorImageIndex++] = intensity;
                    depthImageBuffer[colorImageIndex++] = intensity;
                    depthImageBuffer[colorImageIndex++] = intensity;
                }
                else
                {
                    colorImageBuffer[colorImageIndex + 0] = 0;
                    colorImageBuffer[colorImageIndex + 1] = 0;
                    colorImageBuffer[colorImageIndex + 2] = 0;

                    depthImageBuffer[colorImageIndex++] = 0;
                    depthImageBuffer[colorImageIndex++] = 0;
                    depthImageBuffer[colorImageIndex++] = 0;
                }
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
                Mat src_col = BitmapSourceConverter.ToMat(collor);
                Cv2.ImShow("colorimage", src_col);
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
                depthFrameData = BilateralFilter(depthFrameData);
                depthFrameData = BilateralFilter(depthFrameData);
                vertexData = VertexmapCreate(depthFrameData);
                //法線マップの作成
                normalData = NormalmapCreate(vertexData);

                //カラー画像の範囲で法線マップ・頂点マップにマスクをかける
                vertexData = MaskVbyC(colorImageBuffer, vertexData);
                normalData = MaskNbyC(colorImageBuffer, normalData);

                //カラー画像のバイラテラルフィルタ
                colorImageBuffer = Color_bilateral(colorImageBuffer);

                //hueはクラスタリングでも使うので別クラスを定義
                double[] ibuffer = new double[colorImageBuffer.Length];
                double[] hsi = new double[colorImageBuffer.Length];
                var rm_color = new byte[colorImageBuffer.Length];
                ibuffer = Ispace_fromRGB(colorImageBuffer, ibuffer);
                hsi = Hsi_fromRGB(ibuffer, hsi);
                //鏡面反射除去
                rm_color = Remove_specular(ibuffer, hsi, colorImageBuffer, rm_color);

                //鏡面反射除去画像を用いてクラスタリング（アルベド推定）
                Km_colpos(vertexData, rm_color);

                //Sfimage_miyazaki(ibuffer, rm_color);//宮崎先生の手法

                /*夏やること
                 * 1．拡散反射成分と頂点画像からアルベドの推定
                 *    ▶デプスの閾値を決める（近いところは信頼度が高い）
                 *    その範囲内でクラスタリングを行う（計算範囲減）
                 * 2．中央値フィルタの実装*/

                //test();
                //カラー画像のクラスタリング
                Kmeans_segmentation(colorImageBuffer, vertexData);
                //Km_hsi(hsi, normalData, colorImageBuffer);
            }
            colorFrame.Dispose();
            depthFrame.Dispose();
        }

        private int[] MaskVbyC(byte[] colorImageBuffer , int[] vertexdata)
        {
            int index = 0;
            for (int i = 0; i < depthFrameData.Length; i++)
            {
                index = i * 3;
                if (colorImageBuffer[index] == 0 && 
                    colorImageBuffer[index + 1] == 0 && 
                    colorImageBuffer[index + 2] == 0)
                {
                    vertexdata[index] = 0;
                    vertexdata[index + 1] = 0;
                    vertexdata[index + 2] = 0;
                }
            }
            return vertexdata;
        }

        private double[,] MaskNbyC(byte[] colorImageBuffer, double[,] normaldata)
        {
            int index = 0;
            for (int i = 0; i < depthFrameData.Length; i++)
            {
                index = i * 3;
                if (colorImageBuffer[index] == 0 &&
                    colorImageBuffer[index + 1] == 0 &&
                    colorImageBuffer[index + 2] == 0)
                {
                    normaldata[i,0] = 0;
                    normaldata[i,1] = 0;
                    normaldata[i,2] = 0;
                }
            }
            return normaldata;
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

        /*=========================================頂点マップから法線マップを作成する関数(Holzerらの手法)======*/
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

        //配列用テスト(頂点差からとったベクトルを方向ベクトルに正規化)
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

        //カラー画像平滑化（バイラテラルフィルタ）
        private byte[] Color_bilateral(byte[] colorImageData)
        {
            int uw = 0;//uv座標 width
            int uh = 0;//       height
            double a = -0.01;
            double b = -0.1;
            byte[] smoothincolor = new byte[colorImageData.Length];
            for (int i = 0; i < depthFrameData.Length; i++)
            {
                uw = i % depthFrameDescription.Width;
                uh = i / depthFrameDescription.Width;
                int bgrIndex = i * (int)colorFrameDescription.BytesPerPixel;
                float r_nume = 0; //nume 分子
                float r_done = 0; //done 分母
                float g_nume = 0;
                float g_done = 0;
                float b_nume = 0;
                float b_done = 0;
                //5*5のフィルタ
                for (int m = -2; m < 3; m++)//h
                {
                    for (int n = -2; n < 3; n++)//w
                    {
                        if (uh == (i - 2) / depthFrameDescription.Width && uh == (i + 2) / depthFrameDescription.Width
                            && uw - 2 > 0 && uw + 2 < depthFrameDescription.Width && uh + 2 < depthFrameDescription.Height && uh - 2 > 0)
                        {
                            float dis = m * m + n * n;
                            double b_diff = Math.Pow(colorImageData[bgrIndex + 0] - colorImageData[bgrIndex + m * depthFrameDescription.Width * colorFrameDescription.BytesPerPixel + n * colorFrameDescription.BytesPerPixel + 0], 2);
                            double g_diff = Math.Pow(colorImageData[bgrIndex + 1] - colorImageData[bgrIndex + m * depthFrameDescription.Width * colorFrameDescription.BytesPerPixel + n * colorFrameDescription.BytesPerPixel + 1], 2);
                            double r_diff = Math.Pow(colorImageData[bgrIndex + 2] - colorImageData[bgrIndex + m * depthFrameDescription.Width * colorFrameDescription.BytesPerPixel + n * colorFrameDescription.BytesPerPixel + 2], 2);
                            b_nume += (float)(colorImageData[bgrIndex + m * depthFrameDescription.Width * colorFrameDescription.BytesPerPixel + n * colorFrameDescription.BytesPerPixel] * Math.Exp(dis * a) * Math.Exp(b_diff * b));
                            b_done += (float)(Math.Exp(dis * a) * Math.Exp(b_diff * b));
                            g_nume += (float)(colorImageData[bgrIndex + m * depthFrameDescription.Width * colorFrameDescription.BytesPerPixel + n * colorFrameDescription.BytesPerPixel + 1] * Math.Exp(dis * a) * Math.Exp(g_diff * b));
                            g_done += (float)(Math.Exp(dis * a) * Math.Exp(g_diff * b));
                            r_nume += (float)(colorImageData[bgrIndex + m * depthFrameDescription.Width * colorFrameDescription.BytesPerPixel + n * colorFrameDescription.BytesPerPixel + 2] * Math.Exp(dis * a) * Math.Exp(r_diff * b));
                            r_done += (float)(Math.Exp(dis * a) * Math.Exp(r_diff * b));
                        }
                    }
                }
                smoothincolor[bgrIndex] = (byte)(b_nume / b_done);
                smoothincolor[bgrIndex + 1] = (byte)(g_nume / g_done);
                smoothincolor[bgrIndex + 2] = (byte)(r_nume / r_done);
            }
            //表示
            //頂点マップの表示
            /*
            BitmapSource vertexMap = BitmapSource.Create(this.depthFrameDescription.Width,
                this.depthFrameDescription.Height,
                96, 96, PixelFormats.Bgr32, null, smoothincolor, this.depthFrameDescription.Width * (int)this.colorFrameDescription.BytesPerPixel);
            Mat src = BitmapSourceConverter.ToMat(vertexMap);
            Cv2.ImShow("colbi", src);
            */
            return smoothincolor;
        }


        void OnClick(object sender, RoutedEventArgs e)
        {
            click = true;
        }

        //Kmeans法によるクラスタリング（色＋頂点座標に改良予定）
        private void Kmeans_segmentation(byte[] colorbuffer, int[] vertexdata)
        {
            //Cv2.Kmeans;
            const int CLASS = 3;
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
                        for (int g = 0; g < CLASS; g++) Debug.WriteLine(centers.At<Vec3f>(g));
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


        #region hsi_km
        //Kmeans法によるクラスタリング（色＋頂点座標に改良予定）
        private void Km_hsi(double[] hsi, double[,] normaldata, byte[] colorbuffer)
        {
            //Cv2.Kmeans;
            const int CLASS = 16;
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
                                    float hue = (float)(hsi[i * colorFrameDescription.BytesPerPixel]);
                                    src.Set<float>(i, hue);
                                    /*
                                    Vec4f vec4f = new Vec4f
                                    {
                                        Item0 = (float)(hsi[i * colorFrameDescription.BytesPerPixel] * (180 / Math.PI)) + 90,//色相
                                        Item1 = (float)(normaldata[index, 0] + 1) * 2,//法線
                                        Item2 = (float)(normaldata[index, 1] + 1) * 2,
                                        Item3 = (float)(normaldata[index, 2]) * 2
                                    };
                                    src.Set<Vec4f>(i, vec4f);
                                    */
                                }
                            }
                        }

                        var criteria = new TermCriteria(type: CriteriaType.Eps | CriteriaType.MaxIter, maxCount: 10, epsilon: 1.0);
                        Cv2.Kmeans(src, CLASS, cluster, criteria, 3, KMeansFlags.PpCenters, centers);
                        for (int g = 0; g < CLASS; g++) Debug.WriteLine(centers.At<Vec4f>(g));
                        i = 0;
                        byte gs = 255 / CLASS;
                        Mat output = new Mat(depthFrameDescription.Height, depthFrameDescription.Width, MatType.CV_8UC3);
                        #region centercol
                        /*
                        //クラスごとのRGBを格納
                        int[] sumcolor = new int[CLASS * 3];
                        int[] classcount = new int[CLASS];
                        for (int j = 0; j < depthFrameData.Length; i++)
                        {
                            int ind = cluster.Get<int>(i);
                            classcount[ind] += 1;
                            ind *= 3;
                            sumcolor[ind] += colorbuffer[j++];//B
                            sumcolor[ind + 1] += colorbuffer[j++];//G
                            sumcolor[ind + 2] += colorbuffer[j++];//R
                        }
                        byte[] centercol = new byte[CLASS * 3];
                        for (int k = 0; k < CLASS; k++)
                        {
                            int num = k * 3;
                            centercol[num] = (byte)(sumcolor[num] / classcount[k]);
                            centercol[num + 1] = (byte)(sumcolor[num + 1] / classcount[k]);
                            centercol[num + 2] = (byte)(sumcolor[num + 2] / classcount[k]);
                        }
                        */
                        i = 0;
                        for (int y = 0; y < depthFrameDescription.Height; y++)
                        {
                            for (int x = 0; x < depthFrameDescription.Width; x++, i++)
                            {
                                //Vec3b col = new Vec3b();
                                //0～7のクラスが割り当てられている
                                int ind = cluster.Get<int>(i);
                                //if (ind > 7 || ind < 0) Debug.WriteLine(ind + "sita");
                                /*
                                col[0] = centercol[ind];    //B
                                col[1] = centercol[ind + 1];//G
                                col[2] = centercol[ind + 2];//R
                                */

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
                        #endregion
                        Cv2.ImShow("km_2", output);
                        //Debug.WriteLine(colorFrameDescription.BytesPerPixel);
                    }
                }
            }
        }

        //Kmeans法によるクラスタリング（色＋頂点座標に改良予定）
        private void Km_colpos(int[] vertexdata, byte[] rscolor)
        {
            //とりあえず2つの物体で考える
            const int CLASS = 3;
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
                                
                                    /*
                                    Vec4f vec4f = new Vec4f
                                    {
                                        Item0 = (float)(hsi[i * colorFrameDescription.BytesPerPixel] * (180 / Math.PI)) + 90,//色相
                                        Item1 = (float)(normaldata[index, 0] + 1) * 2,//法線
                                        Item2 = (float)(normaldata[index, 1] + 1) * 2,
                                        Item3 = (float)(normaldata[index, 2]) * 2
                                    };
                                    src.Set<Vec4f>(i, vec4f);
                                    */
                                
                            }
                        }

                        var criteria = new TermCriteria(type: CriteriaType.Eps | CriteriaType.MaxIter, maxCount: 10, epsilon: 1.0);
                        Cv2.Kmeans(src, CLASS, cluster, criteria, 3, KMeansFlags.PpCenters, centers);
                        for (int g = 0; g < CLASS; g++) Debug.WriteLine(centers.At<Vec4f>(g));
                        i = 0;
                        byte gs = 255 / CLASS;
                        Mat output = new Mat(depthFrameDescription.Height, depthFrameDescription.Width, MatType.CV_8UC3);
                        #region centercol
                        /*
                        //クラスごとのRGBを格納
                        int[] sumcolor = new int[CLASS * 3];
                        int[] classcount = new int[CLASS];
                        for (int j = 0; j < depthFrameData.Length; i++)
                        {
                            int ind = cluster.Get<int>(i);
                            classcount[ind] += 1;
                            ind *= 3;
                            sumcolor[ind] += colorbuffer[j++];//B
                            sumcolor[ind + 1] += colorbuffer[j++];//G
                            sumcolor[ind + 2] += colorbuffer[j++];//R
                        }
                        byte[] centercol = new byte[CLASS * 3];
                        for (int k = 0; k < CLASS; k++)
                        {
                            int num = k * 3;
                            centercol[num] = (byte)(sumcolor[num] / classcount[k]);
                            centercol[num + 1] = (byte)(sumcolor[num + 1] / classcount[k]);
                            centercol[num + 2] = (byte)(sumcolor[num + 2] / classcount[k]);
                        }
                        */
                        i = 0;
                        for (int y = 0; y < depthFrameDescription.Height; y++)
                        {
                            for (int x = 0; x < depthFrameDescription.Width; x++, i++)
                            {
                                //Vec3b col = new Vec3b();
                                //0～7のクラスが割り当てられている
                                int ind = cluster.Get<int>(i);
                                //if (ind > 7 || ind < 0) Debug.WriteLine(ind + "sita");
                                /*
                                col[0] = centercol[ind];    //B
                                col[1] = centercol[ind + 1];//G
                                col[2] = centercol[ind + 2];//R
                                */

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
                        #endregion
                        Cv2.ImShow("km", output);
                        //Debug.WriteLine(colorFrameDescription.BytesPerPixel);
                    }
                }
            }
        }

            #endregion hsi_km

            #region remove_specular
            //RGB to Ispace
            private double[] Ispace_fromRGB(byte[] colorbuffer, double[] ibuffer)
        {
            //RGB　⇒　Ix, Iy, Iz
            uint index = 0;
            for (uint i = 0; i < depthFrameData.Length; i++)
            {
                index = i * colorFrameDescription.BytesPerPixel;
                if (colorbuffer[index] != colorbuffer[index + 1] && colorbuffer[index + 1] != colorbuffer[index + 2])
                {
                    ibuffer[index] = colorbuffer[index + 2] - 0.5 * colorbuffer[index + 1] - 0.5 * colorbuffer[index];   //Ix
                    ibuffer[index + 1] = (colorbuffer[index + 1] - colorbuffer[index]) * 0.5 * Math.Sqrt(3);             //Iy
                    ibuffer[index + 2] = (double)colorbuffer[index + 2] / 3 + (double)colorbuffer[index + 1] / 3 + (double)colorbuffer[index] / 3;//Iz   
                }
            }
            return ibuffer;
        }
        //Ispace to HSI
        private double[] Hsi_fromRGB(double[] ibuffer, double[] hsi)
        {
            //Ix, Iy, Iz ⇒　HSI
            uint index = 0;
            for (uint j = 0; j < depthFrameData.Length; j++)
            {
                index = j * colorFrameDescription.BytesPerPixel;
                if (ibuffer[index] != 0)
                {
                    hsi[index] = (int)(Math.Atan(ibuffer[index + 1] / ibuffer[index]) * (180 / Math.PI) + 90);            //hue 後に番号として使うので整数にする
                    hsi[index + 1] = Math.Sqrt(ibuffer[index] * ibuffer[index] + ibuffer[index + 1] * ibuffer[index + 1]);//saturation
                    hsi[index + 2] = ibuffer[index + 2];                                                                  //intensity
                }
            }
            return hsi;
        }

        private byte[] Remove_specular(double[] ibuffer, double[] hsi, byte[] colorbuffer, byte[] rm_color)
        {
            var hsit = new List<List<Vec2d>>();
            hsit.Add(new List<Vec2d>());
            //hueを 24分割にしてみる
            for (int i = 0; i < 24; i++)
            {
                hsit.Add(new List<Vec2d>());
            }
            uint index = 0;
            for (uint j = 0; j < depthFrameData.Length; j++)
            {
                index = j * colorFrameDescription.BytesPerPixel;
                if (ibuffer[index] != 0)//これでok
                {
                    Vec2d si = new Vec2d(hsi[index + 1], hsi[index + 2]);
                    //hueの分割数で変化
                    hsit[(int)(hsi[index] / 15.0 * 2)].Add(si);
                }
            }
            //最小二乗法による傾きを格納・そのための二乗和と積和
            double[] tilt = new double[depthFrameDescription.LengthInPixels];       //傾き
            double[] squares_Sum = new double[depthFrameDescription.LengthInPixels];//二乗和
            double[] multipl_Sum = new double[depthFrameDescription.LengthInPixels];//積和
            //hueごとにsaturationで昇順＋同じ場合はintensityで昇順
            for (int k = 0; k < 24; k++)
            {
                //var sorted = new List<Vec2d>(hsit[k].Count);
                var sorted = hsit[k].OrderBy(e => e[0]).ThenBy(e => e[1]).ToList();
                //saturationの値が同じとき最小のintensityに書き換えておく
                for (int l = 0; l < sorted.Count(); l++)
                {
                    //saturationに対する最小のintensity
                    if (l > 0 && sorted[l - 1][0] == sorted[l][0])
                    {
                        sorted[l] = sorted[l - 1];
                    }
                    squares_Sum[k] += sorted[l][0] * sorted[l][0];
                    multipl_Sum[k] += sorted[l][0] * sorted[l][1];
                }
                //hueごとの傾きを求める
                tilt[k] = multipl_Sum[k] / squares_Sum[k];
            }
            //求めた傾きでhsi[]のintensityの値だけ計算し直す
            for (uint m = 0; m < depthFrameData.Length; m++)
            {
                index = m * colorFrameDescription.BytesPerPixel;
                if (ibuffer[index] != 0)
                {
                    //hueの分割によって変形
                    hsi[index + 2] = tilt[(int)(hsi[index] / 15.0 * 2)] * hsi[index + 1];//intensity
                }
            }

            //hsiからカラー画像(rgb)への逆変換
            double ix, iy, iz = 0;
            double b, g, r = 0;
            double max = 0;
            byte[] rmc = new byte[3];
            for (uint j = 0; j < depthFrameData.Length; j++)
            {
                index = j * colorFrameDescription.BytesPerPixel;
                if (colorbuffer[index] != colorbuffer[index + 1] && colorbuffer[index + 1] != colorbuffer[index + 2])
                {
                    hsi[index] = (hsi[index] - 90) * Math.PI / 180;
                    if (ibuffer[index] < 0 && ibuffer[index + 1] < 0)
                    {
                        ix = hsi[index + 1] * Math.Cos(hsi[index]) * -1;
                        iy = hsi[index + 1] * Math.Sin(hsi[index]) * -1;
                        iz = hsi[index + 2];
                    }
                    else
                    {
                        ix = hsi[index + 1] * Math.Cos(hsi[index]);
                        iy = hsi[index + 1] * Math.Sin(hsi[index]);
                        iz = hsi[index + 2];
                    }
                    //BGRの順
                    b = iz - ix / 3 - iy / Math.Sqrt(3);
                    g = iz - ix / 3 + iy / Math.Sqrt(3);
                    r = iz + ix * 2 / 3;
                    
                    Pixel_normaliz(rmc, b, g, r);
                    rm_color[index] = rmc[0];
                    rm_color[index + 1] = rmc[1];
                    rm_color[index + 2] = rmc[2];
                    /*
                    max = Math.Max(b, Math.Max(g, r));
                    if (max > 255)
                    {
                        Pixel_normaliz(rmc, b, g, r);
                        rm_color[index] = rmc[0];
                        rm_color[index + 1] = rmc[1];
                        rm_color[index + 2] = rmc[2];
                    }
                    else
                    {
                        rm_color[index] = (byte)b;
                        rm_color[index + 1] = (byte)g;
                        rm_color[index + 2] = (byte)r;
                    }*/
                }
                else
                {
                    rm_color[index] = colorbuffer[index];
                    rm_color[index + 1] = colorbuffer[index + 1];
                    rm_color[index + 2] = colorbuffer[index + 2];
                }
            }
            //表示
            BitmapSource rem_color = BitmapSource.Create(this.depthFrameDescription.Width,
            this.depthFrameDescription.Height,
            96, 96, PixelFormats.Bgr32, null, rm_color, this.depthFrameDescription.Width * (int)this.colorFrameDescription.BytesPerPixel);
            Mat src = BitmapSourceConverter.ToMat(rem_color);
            Cv2.ImShow("re_col", src);
            return rm_color;
        }

        private byte[] Pixel_normaliz(byte[] rmc, double b, double g, double r)
        {
            double max = Math.Max(b, Math.Max(g, r));
            rmc[0] = (byte)(b / max * 255);
            rmc[1] = (byte)(g / max * 255);
            rmc[2] = (byte)(r / max * 255);
            return rmc;
        }

        private byte[] Sfimage_miyazaki(double[] ibuffer, byte[] rm_color)
        {
            double[] mbuffer = new double[ibuffer.Length];
            double a = 1.0;
            double b, g, r = 0;
            byte[] mrmc = new byte[3];
            for (int i = 0; i < depthFrameData.Length; i++)
            {
                int index = i * (int)colorFrameDescription.BytesPerPixel;
                mbuffer[index + 0] = ibuffer[index + 0];
                mbuffer[index + 1] = ibuffer[index + 1];
                mbuffer[index + 2] = a * Math.Sqrt(ibuffer[index] * ibuffer[index] + ibuffer[index + 1] * ibuffer[index + 1]);
            }
            for (int i = 0; i < depthFrameData.Length; i++)
            {
                int index = i * (int)colorFrameDescription.BytesPerPixel;
                b = mbuffer[index + 2] - mbuffer[index] / 3 - mbuffer[index + 1] / Math.Sqrt(3);
                g = mbuffer[index + 2] - mbuffer[index] / 3 + mbuffer[index + 1] / Math.Sqrt(3);
                r = mbuffer[index + 2] + mbuffer[index] * 2 / 3;
                Pixel_normaliz(mrmc, b, g, r);
                rm_color[index] = mrmc[0];
                rm_color[index + 1] = mrmc[1];
                rm_color[index + 2] = mrmc[2];
            }
            BitmapSource rem_color = BitmapSource.Create(this.depthFrameDescription.Width,
            this.depthFrameDescription.Height,
            96, 96, PixelFormats.Bgr32, null, rm_color, this.depthFrameDescription.Width * (int)this.colorFrameDescription.BytesPerPixel);
            Mat src = BitmapSourceConverter.ToMat(rem_color);
            Cv2.ImShow("miyazaki_col", src);
            return rm_color;
        }
        #endregion

        /*=============使わなくなった関数=====================================================================================================*/
        #region disused
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

        private void Remove_highlight(byte[] colorbuffer)
        {
            //BGRの順に注意‼！！！！！！！！！
            var rm_color = new byte[colorbuffer.Length];
            //RGB　⇒　Ix, Iy, Iz
            double[] ibuffer = new double[colorbuffer.Length];
            uint index = 0;
            for (uint i = 0; i < depthFrameData.Length; i++)
            {
                index = i * colorFrameDescription.BytesPerPixel;
                if (colorbuffer[index] != colorbuffer[index + 1] && colorbuffer[index + 1] != colorbuffer[index + 2])
                {
                    ibuffer[index] = colorbuffer[index + 2] - 0.5 * colorbuffer[index + 1] - 0.5 * colorbuffer[index];   //Ix
                    ibuffer[index + 1] = (colorbuffer[index + 1] - colorbuffer[index]) * 0.5 * Math.Sqrt(3);             //Iy
                    ibuffer[index + 2] = (double)colorbuffer[index + 2] / 3 + (double)colorbuffer[index + 1] / 3 + (double)colorbuffer[index] / 3;//Iz   
                }
            }

            //Ix, Iy, Iz ⇒　HSI
            double[] hsi = new double[colorbuffer.Length];
            for (uint j = 0; j < depthFrameData.Length; j++)
            {
                index = j * colorFrameDescription.BytesPerPixel;
                if (ibuffer[index] != 0)
                {
                    hsi[index] = (int)(Math.Atan(ibuffer[index + 1] / ibuffer[index]) * (180 / Math.PI) + 90);            //hue 後に番号として使うので整数にする
                    hsi[index + 1] = Math.Sqrt(ibuffer[index] * ibuffer[index] + ibuffer[index + 1] * ibuffer[index + 1]);//saturation
                    hsi[index + 2] = ibuffer[index + 2];                                                                  //intensity
                }
            }

            //問題：Saturationに対する最小のintensityを導かないといけない
            //sortすると座標が狂うので、hsi[]は座標保持で最終的にRGBまで逆算で求める用
            //hueごとの傾きを求める用にhsitを用いる
            var hsit = new List<List<Vec2d>>();
            hsit.Add(new List<Vec2d>());
            for (int i = 0; i < 180; i++)
            {
                hsit.Add(new List<Vec2d>());
            }
            for (uint j = 0; j < depthFrameData.Length; j++)
            {
                index = j * colorFrameDescription.BytesPerPixel;
                if (ibuffer[index] != 0)//これでok
                {
                    Vec2d si = new Vec2d(hsi[index + 1], hsi[index + 2]);
                    hsit[(int)hsi[index]].Add(si);
                }
            }
            //最小二乗法による傾きを格納・そのための二乗和と積和
            double[] tilt = new double[depthFrameDescription.LengthInPixels];       //傾き
            double[] squares_Sum = new double[depthFrameDescription.LengthInPixels];//二乗和
            double[] multipl_Sum = new double[depthFrameDescription.LengthInPixels];//積和
            //hueごとにsaturationで昇順＋同じ場合はintensityで昇順
            for (int k = 0; k < 180; k++)
            {
                //var sorted = new List<Vec2d>(hsit[k].Count);
                var sorted = hsit[k].OrderBy(e => e[0]).ThenBy(e => e[1]).ToList();
                //saturationの値が同じとき最小のintensityに書き換えておく
                for (int l = 0; l < sorted.Count(); l++)
                {
                    //saturationに対する最小のintensity
                    if (l > 0 && sorted[l - 1][0] == sorted[l][0])
                    {
                        sorted[l] = sorted[l - 1];
                    }
                    squares_Sum[k] += sorted[l][0] * sorted[l][0];
                    multipl_Sum[k] += sorted[l][0] * sorted[l][1];
                }
                //hueごとの傾きを求める
                tilt[k] = multipl_Sum[k] / squares_Sum[k];
            }
            //求めた傾きでhsi[]のintensityの値だけ計算し直す
            for (uint m = 0; m < depthFrameData.Length; m++)
            {
                index = m * colorFrameDescription.BytesPerPixel;
                if (ibuffer[index] != 0)
                {
                    hsi[index + 2] = tilt[(int)hsi[index]] * hsi[index + 1];//intensity
                }
            }

            //hsiからカラー画像(rgb)への逆変換
            double ix, iy, iz = 0;
            for (uint j = 0; j < depthFrameData.Length; j++)
            {
                index = j * colorFrameDescription.BytesPerPixel;
                if (colorbuffer[index] != colorbuffer[index + 1] && colorbuffer[index + 1] != colorbuffer[index + 2])
                {
                    hsi[index] = (hsi[index] - 90) * Math.PI / 180;
                    ix = hsi[index + 1] * Math.Cos(hsi[index]);
                    iy = hsi[index + 1] * Math.Sin(hsi[index]);
                    iz = hsi[index + 2];
                    //BGRの順
                    rm_color[index] = (byte)(iz - ix / 3 - iy / Math.Sqrt(3));
                    rm_color[index + 1] = (byte)(iz - ix / 3 + iy / Math.Sqrt(3));
                    rm_color[index + 2] = (byte)(iz + ix * 2 / 3);
                }
                else
                {
                    rm_color[index] = colorbuffer[index];
                    rm_color[index + 1] = colorbuffer[index + 1];
                    rm_color[index + 2] = colorbuffer[index + 2];
                }
            }
            //表示
            BitmapSource rem_color = BitmapSource.Create(this.depthFrameDescription.Width,
            this.depthFrameDescription.Height,
            96, 96, PixelFormats.Bgr32, null, rm_color, this.depthFrameDescription.Width * (int)this.colorFrameDescription.BytesPerPixel);
            Mat src = BitmapSourceConverter.ToMat(rem_color);
            Cv2.ImShow("re_col", src);
        }

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

        //カラー画像平滑化（バイラテラルフィルタ）めちゃ重
        private void OpenCV_bilateral(BitmapSource colorImage)
        {
            int d = 9;
            double sigmaColor = 75.0;
            double sigmaSpace = 750.0;
            Mat src = BitmapSourceConverter.ToMat(colorImage);
            src = src.CvtColor(ColorConversionCodes.BGRA2BGR);
            using (var dst = new Mat())
            {
                var dst2 = new Mat();
                Cv2.BilateralFilter(src, dst, d, sigmaColor, sigmaSpace);
                Cv2.BilateralFilter(dst, dst2, d, sigmaColor, sigmaSpace);
                Cv2.ImShow("col_bi", dst);
            };
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
        #endregion
    }
}
