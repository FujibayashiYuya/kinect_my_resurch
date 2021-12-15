using Microsoft.Kinect;
using OpenCvSharp;
using OpenCvSharp.WpfExtensions;
using System;
using System.IO;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Runtime.InteropServices;
using System.Drawing;
using System.Drawing.Imaging;

//openTk用
using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL;
using OpenTK.Input;
//using PixelFormat = OpenTK.Graphics.OpenGL.PixelFormat;

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

        //マップの幅
        public static int map_w = 128;
        public static int map_h = 128;
        //放射照度マップ
        public static double[,] irradiancemap = new double[128 * 128, 2];//傾きとy切片を格納

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
                MessageBox.Show("Kinectの検出できません");
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

            //マスク用
            var mask = new byte[depthFrameDescription.LengthInPixels];
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
                if (depthFrameData[i] < 1000)
                {
                    colorImageBuffer[colorImageIndex + 0] = colorFrameData[colorBufferIndex + 0];//B
                    colorImageBuffer[colorImageIndex + 1] = colorFrameData[colorBufferIndex + 1];//G
                    colorImageBuffer[colorImageIndex + 2] = colorFrameData[colorBufferIndex + 2];//R

                    //深度画像
                    byte intensity = (byte)(depthFrameData[i] % 255);
                    depthImageBuffer[colorImageIndex++] = intensity;
                    depthImageBuffer[colorImageIndex++] = intensity;
                    depthImageBuffer[colorImageIndex++] = intensity;

                    //マスク
                    mask[i] = 1;
                }
                else
                {
                    colorImageBuffer[colorImageIndex + 0] = 0;
                    colorImageBuffer[colorImageIndex + 1] = 0;
                    colorImageBuffer[colorImageIndex + 2] = 0;

                    depthImageBuffer[colorImageIndex++] = 0;
                    depthImageBuffer[colorImageIndex++] = 0;
                    depthImageBuffer[colorImageIndex++] = 0;

                    mask[i] = 0;
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
                var km_img = new byte[colorImageBuffer.Length];
                ibuffer = Ispace_fromRGB(colorImageBuffer, ibuffer);
                hsi = Hsi_fromIspace(ibuffer, hsi);
                //鏡面反射除去
                rm_color = Remove_specular(ibuffer, hsi, colorImageBuffer, rm_color);

                //鏡面反射除去画像を用いてクラスタリング（アルベド推定）
                km_img = Km_colpos(vertexData, rm_color, mask, km_img);

                //Sfimage_miyazaki(ibuffer, rm_color);//宮崎先生の手法

                //test();
                //カラー画像のクラスタリング(課題：km_imgにあった戻り値にする)
                //Kmeans_segmentation(colorImageBuffer, vertexData, km_img);
                //Km_hsi(hsi, normalData, colorImageBuffer);
                //放射照度マップの作成
                //引数：拡散反射画像　、　クラスタリング画像　、　法線画像
                irradiancemap = MakeIrradiancemapBySaisyounijou(rm_color, km_img, normalData, mask);

                //最期に表示
                opentk();

            }
            colorFrame.Dispose();
            depthFrame.Dispose();
        }

        private int[] MaskVbyC(byte[] colorImageBuffer, int[] vertexdata)
        {
            int index = 0;
            for (int i = 0; i < depthFrameData.Length; i++)
            {
                index = (int)(i * colorFrameDescription.BytesPerPixel);
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
                index = (int)(i * colorFrameDescription.BytesPerPixel);
                if (colorImageBuffer[index] == 0 &&
                    colorImageBuffer[index + 1] == 0 &&
                    colorImageBuffer[index + 2] == 0)
                {
                    normaldata[i, 0] = 0;
                    normaldata[i, 1] = 0;
                    normaldata[i, 2] = 0;
                }
            }

            //法線取得状況を確認
            //画像に出力
            var normalImage = new byte[depthFrameDescription.LengthInPixels * colorFrameDescription.BytesPerPixel];
            for (int i = 0; i < depthFrameData.Length; ++i)
            {
                int normalImageIndex = (int)(i * colorFrameDescription.BytesPerPixel);
                //頂点座標
                normalImage[normalImageIndex + 2] = (byte)((normaldata[i, 0] + 1) * 127.5);//x R
                normalImage[normalImageIndex + 1] = (byte)((normaldata[i, 1] + 1) * 127.5);//y G
                normalImage[normalImageIndex + 0] = (byte)(normaldata[i, 2] * 255);//z B
            }
            //頂点マップの表示
            BitmapSource vertexMap = BitmapSource.Create(this.depthFrameDescription.Width,
                this.depthFrameDescription.Height,
                96, 96, PixelFormats.Bgr32, null, normalImage, this.depthFrameDescription.Width * (int)this.colorFrameDescription.BytesPerPixel);
            Mat src = BitmapSourceConverter.ToMat(vertexMap);
            Cv2.ImShow("normal2", src);

            return normaldata;
        }

        /*=====カラー画像の中央値フィルタ（RGB⇒グレースケール）======優先順位低い*/
        private byte[] Medianfilta(byte[] colorbuffer)
        {
            byte[] grayim = new byte[depthFrameData.Length];
            byte[] medianim = new byte[colorbuffer.Length];
            long index = 0;
            for (int i = 0; i < depthFrameData.Length; i++)
            {
                index = i * colorFrameDescription.BytesPerPixel;
                //範囲外の0は計算から外す
                if (colorbuffer[index] != 0 && colorbuffer[index + 1] != 0 && colorbuffer[index + 2] != 0)
                {
                    //gray = R * 0.3 + G * 0.59 + B * 0.11
                    grayim[i] = (byte)(colorbuffer[index] * 0.11 + colorbuffer[index + 1] * 0.59 + colorbuffer[index + 2] * 0.3);
                }
            }
            //grayで中央値の箇所をカラー画像の画素値で置き換える

            return medianim;
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
                //var range_x = depthFrameDescription.Width / calibrationData.FocalLengthX * 4500;
                //var range_y = depthFrameDescription.Height / calibrationData.FocalLengthY * 4500;

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
                vertexImage[colorImageIndex] = (byte)(vertexData[colorImageIndex] / range_x * 255);//x B
                vertexImage[colorImageIndex + 1] = (byte)(vertexData[colorImageIndex + 1] / range_y * 255);//y G
                vertexImage[colorImageIndex + 2] = (byte)(255 * (vertexData[colorImageIndex + 2] - 500) / 7500);//z R
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
                vy[0] = (VertexData[py0] - VertexData[py1]) * 0.5;
                vy[1] = (VertexData[++py0] - VertexData[++py1]) * 0.5;
                vy[2] = (VertexData[++py0] - VertexData[++py1]) * 0.5;
                //↑でおかしい？
                if (vx[0] != 0 || vx[1] != 0 || vx[2] != 0)//まとめるとNoNがでてきた
                {
                    if (vy[0] != 0 || vy[1] != 0 || vy[2] != 0)
                    {
                        n = VecNormalized(VecCross(vx, vy));//カメラ方向が＋Zになるように
                    }
                }
                normalData[i, 0] = n[0];
                normalData[i, 1] = n[1];
                normalData[i, 2] = n[2];
                //Debug.WriteLine(n[0] + " " + n[1] + " " + n[2]);// + " " + vx[0] + vx[1] + vx[2] + " " + vy[0]+vy[1]+vy[2]);
            }

            //画像に出力
            var normalImage = new byte[depthFrameDescription.LengthInPixels * colorFrameDescription.BytesPerPixel];
            for (int i = 0; i < depthFrameData.Length; ++i)
            {
                int normalImageIndex = (int)(i * colorFrameDescription.BytesPerPixel);
                //頂点座標
                normalImage[normalImageIndex + 2] = (byte)((normalData[i, 0] + 1) * 127.5);//x R
                normalImage[normalImageIndex + 1] = (byte)((normalData[i, 1] + 1) * 127.5);//y G
                normalImage[normalImageIndex + 0] = (byte)(normalData[i, 2] * 255);//z B
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
        private double[] Hsi_fromIspace(double[] ibuffer, double[] hsi)
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

                    //RGBの内最大値が255になるように正規化
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
                        for (int g = 0; g < CLASS; g++) Debug.WriteLine(centers.At<Vec6f>(g));
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

        //Kmeans法によるクラスタリング（頂点色＋頂点座標に改良予定）
        //問題点：範囲外の黒い部分も対象になっている
        //対策：マスクを用意する（mask[i] = 1の時だけKmeasnで計算、あとからmask[i] = 1の箇所に上から順に画素値を代入させる）
        private byte[] Km_colpos(int[] vertexdata, byte[] rscolor, byte[] mask, byte[] km_img)
        {
            //とりあえず2つの物体で考える
            const int CLASS = 2;
            int srcbuffercount = 0;
            for (int m = 0; m < mask.Length; m++)
            {
                if (mask[m] == 1) srcbuffercount++;
            }
            //色(もともとByte)＋頂点座標(int型)の6chのMat(★float型じゃないとダメ)
            using (Mat src = new Mat(srcbuffercount, 1, MatType.CV_32FC(6)))
            {
                using (Mat cluster = new Mat())
                {
                    using (Mat centers = new Mat(CLASS, 1, MatType.CV_32FC(4)))
                    {
                        int i = 0;
                        int n = 0;
                        long index = 0;
                        //引数byte[]をKmeans()に適した形にする
                        for (int y = 0; y < depthFrameDescription.Height; y++)
                        {
                            for (int x = 0; x < depthFrameDescription.Width; x++, i++)
                            {
                                if (mask[i] == 1)
                                {
                                    index = i * colorFrameDescription.BytesPerPixel;
                                    Vec6f vec6f = new Vec6f
                                    {
                                        Item0 = (float)(rscolor[index]),
                                        Item1 = (float)(rscolor[index + 1]),
                                        Item2 = (float)(rscolor[index + 2]),
                                        Item3 = (float)(vertexdata[index]),
                                        Item4 = (float)(vertexdata[index + 1]),
                                        Item5 = (float)(vertexdata[index + 2])
                                    };
                                    src.Set<Vec6f>(n, vec6f);
                                    n++;
                                }
                            }
                        }

                        var criteria = new TermCriteria(type: CriteriaType.Eps | CriteriaType.MaxIter, maxCount: 10, epsilon: 1.0);
                        Cv2.Kmeans(src, CLASS, cluster, criteria, 3, KMeansFlags.PpCenters, centers);
                        for (int g = 0; g < CLASS; g++) Debug.WriteLine(centers.At<Vec6f>(g));
                        i = 0;
                        n = 0;
                        int kmindex = 0;
                        Mat output = new Mat(depthFrameDescription.Height, depthFrameDescription.Width, MatType.CV_8UC3);
                        for (int y = 0; y < depthFrameDescription.Height; y++)
                        {
                            for (int x = 0; x < depthFrameDescription.Width; x++, i++)
                            {
                                Vec3b col = new Vec3b();
                                kmindex = (int)((y * depthFrameDescription.Width + x) * colorFrameDescription.BytesPerPixel);
                                //その座標のマスクの値が０の時RGBは０、マスクの値が1の時はクラスター中心の値を代入
                                if (mask[i] == 0)
                                {
                                    col[0] = 0;//B
                                    col[1] = 0;//G
                                    col[2] = 0;//R
                                    output.Set<Vec3b>(y, x, col);

                                    //データ用配列
                                    km_img[kmindex] = 0;
                                    km_img[kmindex + 1] = 0;
                                    km_img[kmindex + 2] = 0;
                                }
                                else if (mask[i] == 1)
                                {
                                    int ind = cluster.Get<int>(n);

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

                                    km_img[kmindex] = (byte)firstComponent;
                                    km_img[kmindex + 1] = (byte)secondComponent;
                                    km_img[kmindex + 2] = (byte)thirdComponent;
                                    n++;
                                }
                            }
                        }

                        Cv2.ImShow("km", output);
                        //Debug.WriteLine(colorFrameDescription.BytesPerPixel);
                        //頂点マップの表示
                        BitmapSource kms = BitmapSource.Create(this.depthFrameDescription.Width,
                            this.depthFrameDescription.Height,
                            96, 96, PixelFormats.Bgr32, null, km_img, this.depthFrameDescription.Width * (int)this.colorFrameDescription.BytesPerPixel);
                        Mat km = BitmapSourceConverter.ToMat(kms);
                        Cv2.ImShow("km2", km);
                    }
                }
            }
            return km_img;
        }
        #endregion

        //RGB版でやってみる
        private void MakeIrradiancemap(byte[] rm_color, byte[] km_img, double[,] normalData, byte[] mask)
        {
            //マップの幅
            int map_w = 32;
            int map_h = 32;
            //放射照度マップ
            double[] irradiancemap = new double[map_w * map_h];
            int irrad = 0;
            //i:画像の座標
            int i = 0;
            int rgbindex = 0;
            int r_dif = 0, g_dif = 0, b_dif = 0;
            //マップの座標
            int u = 0, v = 0;
            //明度でrm_color - km_imgの値をnormalDataから法線情報を基に放射照度マップに格納する
            //rm_colorの画像座標に対してループ
            for (i = 0; i < depthFrameDescription.LengthInPixels; i++)
            {
                rgbindex = i * 3;
                b_dif = rm_color[rgbindex] - km_img[rgbindex];
                g_dif = rm_color[rgbindex + 1] - km_img[rgbindex + 1];
                r_dif = rm_color[rgbindex + 2] - km_img[rgbindex + 2];
                //放射照度の変化度
                irrad = (b_dif + g_dif + r_dif) / 3;
            }
        }

        //アルベドに対する拡散反射成分の明度の最小二乗法で行う
        private double[,] MakeIrradiancemapBySaisyounijou(byte[] rm_color, byte[] km_img, double[,] normalData, byte[] mask)
        {
            double a = 0, b = 0;
            int rgbindex = 0;
            //マップの座標
            int u = 0, v = 0, uvind = 0;

            double intens_rm = 0, intens_km = 0;
            //最小二乗法用のデータ配列
            int[] datanum = new int[map_w * map_h];
            double[] x_sum = new double[irradiancemap.Length];
            double[] y_sum = new double[irradiancemap.Length];
            double[] xy_sum = new double[irradiancemap.Length];
            double[] xx_sum = new double[irradiancemap.Length];
            //明度でrm_color - km_imgの値をnormalDataから法線情報を基に放射照度マップに格納する
            //rm_colorの画像座標に対してループ
            for (int i = 0; i < depthFrameDescription.LengthInPixels; i++)
            {
                if (mask[i] == 1)
                {
                    rgbindex = (int)(i * colorFrameDescription.BytesPerPixel);
                    //明度を求める(OpenTK用に0～１に正規化)
                    intens_rm = (double)(rm_color[rgbindex] + rm_color[rgbindex + 1] + rm_color[rgbindex + 2]) / 3 / 255; //y
                    intens_km = (double)(km_img[rgbindex] + km_img[rgbindex + 1] + km_img[rgbindex + 2]) / 3 / 255; //x
                    //法線方向からuv座標を求める(11月11日訂正　vの式 n -1 から　1- n)
                    u = (int)((normalData[i, 0] + 1) * (map_w - 1) / 2);
                    v = (int)((1 - normalData[i, 1]) * (map_h - 1) / 2);
                    uvind = v * map_w + u;
                    //最小二乗法に用いる値を代入していく
                    //エラー：インデックスの配列外です。(上でuv座標をW-1にした。0から31行列の32*32の行列)
                    datanum[uvind] += 1;
                    x_sum[uvind] += intens_km;
                    y_sum[uvind] += intens_rm;
                    xy_sum[uvind] += intens_rm * intens_km;
                    xx_sum[uvind] += intens_km * intens_km;
                }
            }

            //最小二乗法を行う
            for (int j = 0; j < datanum.Length; j++)
            {
                //データのない箇所は除外
                if (datanum[j] != 0)
                {
                    a = (datanum[j] * xy_sum[j] - x_sum[j] * y_sum[j]) / (datanum[j] * xx_sum[j] - x_sum[j] * x_sum[j]);
                    b = (xx_sum[j] * y_sum[j] - x_sum[j] * xy_sum[j]) / (datanum[j] * xx_sum[j] - x_sum[j] * x_sum[j]);
                    
                    if(Double.IsNaN(a) || Double.IsInfinity(a))
                    {
                        a = 0;
                        b = 0;
                    }
                    
                    irradiancemap[j, 0] = a;
                    irradiancemap[j, 1] = b;
                    
                }
            }
            /*
            for (int i = 0; i < 32 * 32; i++)
            {
                Debug.WriteLine(irradiancemap[i,0] + " , " + irradiancemap[i, 1]);
            }
            */
            

            byte[] irmap = new byte[irradiancemap.Length * colorFrameDescription.BytesPerPixel];
            for (int j = 0; j < datanum.Length; j++)
            {
                int index = (int)(j * colorFrameDescription.BytesPerPixel);
                if (irradiancemap[j, 0] == 0 && irradiancemap[j, 1] == 0)
                {
                    irmap[index] = 0;
                    irmap[index + 1] = 0;
                    irmap[index + 2] = 0;
                }
                else
                {
                    int color = (int)(255 * irradiancemap[j, 0] + irradiancemap[j, 1] * 255);
                    //とりあえず計算されているところは色表示
                    if (color < 255 || color < 0)
                    {
                        irmap[index] = 255;
                        irmap[index + 1] = 255;
                        irmap[index + 2] = 255;
                    }
                    else
                    {
                        irmap[index] = (byte)color;
                        irmap[index + 1] = (byte)color;
                        irmap[index + 2] = (byte)color;
                    }
                }
            }
            BitmapSource iramap = BitmapSource.Create(map_w,
                            map_h,
                            96, 96, PixelFormats.Bgr32, null, irmap, map_w * (int)this.colorFrameDescription.BytesPerPixel);
            Mat testmap = BitmapSourceConverter.ToMat(iramap);
            Cv2.ImShow("放射照度マップ", testmap);


            return irradiancemap;
        }

        public void opentk()
        {
            using (Game window = new Game())
            {
                window.Run(30.0);
            }
        }
        /*=============使わなくなった関数=====================================================================================================*/
        #region disused
        //補間関数
        public double[,] Interpolation(double[,] irradiancemap)
        {
            //横方向で補間（とりあえず）(一番外側は内挿補間できない)
            for (int v = 0; v < map_h; v++)
            {
                for (int u = 0; u < map_w; u++)
                {
                    int index = v * map_w + u;
                    //もし値に挟まれている箇所があれば（内挿の補間）(行が同じとき)
                    if (irradiancemap[index, 0] != 0 && (index + 1) / map_w == v)
                    {
                        for (int n = 1; n < map_w - u; n++)
                        {
                            //補間値
                            float rate = 0;
                            float x = 0;
                            //index～index+nの間の値は0
                            if (irradiancemap[index + n, 0] != 0)
                            {
                                //Debug.WriteLine("test");
                                //傾きの補間
                                //補間関数：y = 2x^3 - 3x^2 + 1
                                if (irradiancemap[index, 0] > irradiancemap[index + n, 0])
                                {
                                    for (int m = 1; m < n; m++)
                                    {
                                        x = m / n;
                                        rate = 2 * x * x * x - 3 * x * x + 1;
                                        irradiancemap[index + m, 0] = irradiancemap[index, 0] * (1.0 - rate) + irradiancemap[index + n, 0] * rate;
                                    }
                                }
                                else//補間関数：y = -2x^3 + 3x^2
                                {
                                    for (int m = 1; m < n; m++)
                                    {
                                        x = m / n;
                                        rate = -2 * x * x * x + 3 * x * x;
                                        irradiancemap[index + m, 0] = irradiancemap[index, 0] * (1.0 - rate) + irradiancemap[index + n, 0] * rate;
                                    }
                                }

                                //y切片の補間
                                if (irradiancemap[index, 1] > irradiancemap[index + n, 1])
                                {
                                    for (int m = 1; m < n; m++)
                                    {
                                        x = m / n;
                                        rate = 2 * x * x * x - 3 * x * x + 1;
                                        irradiancemap[index + m, 1] = irradiancemap[index, 1] * (1.0 - rate) + irradiancemap[index + n, 1] * rate;
                                    }
                                }
                                else//補間関数：y = -2x^3 + 3x^2
                                {
                                    for (int m = 1; m < n; m++)
                                    {
                                        x = m / n;
                                        rate = -2 * x * x * x + 3 * x * x;
                                        irradiancemap[index + m, 1] = irradiancemap[index, 1] * (1.0 - rate) + irradiancemap[index + n, 1] * rate;
                                    }
                                }
                            }
                            u = u + n;
                        }
                    }
                }
            }
            return irradiancemap;
        }

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

        private System.Numerics.Vector3[] NmC(int[] VertexData)
        {
            var normalData = new int[depthFrameDescription.LengthInPixels * colorFrameDescription.BytesPerPixel];
            var norvecData = new System.Numerics.Vector3[depthFrameDescription.LengthInPixels];
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

        //Kmeans法によるクラスタリング（
        private void Kmeans_segmentation(byte[] colorbuffer, int[] vertexdata, byte[] km_img)
        {
            //Cv2.Kmeans;
            const int CLASS = 5;
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
                        //データ用配列
                        i = 0;


                        //画像用
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
        #endregion
    }

    //以下OpenTK
    struct Vertex
    {
        public OpenTK.Vector3 position;
        public OpenTK.Vector3 normal;
        public OpenTK.Vector2 uv;
        public OpenTK.Vector4 color;

        public Vertex(OpenTK.Vector3 position, OpenTK.Vector3 normal, OpenTK.Vector2 uv, OpenTK.Vector4 color)
        {
            this.position = position;
            this.normal = normal;
            this.uv = uv;
            this.color = color;
        }

        public static readonly int Size = Marshal.SizeOf(default(Vertex));
    }

    public class Game : GameWindow
    {
        #region Camera__Field

        bool isCameraRotating;      //カメラが回転状態かどうか
        OpenTK.Vector2 current, previous;  //現在の点、前の点
        Matrix4 rotate;             //回転行列
        float zoom;                 //拡大度
        float wheelPrevious;        //マウスホイールの前の状態

        #endregion

        Color4 materialAmbient;     //材質の環境光成分
        Color4 materialDiffuse;	//材質の拡散光成分
        Color4 materialSpecular;    //材質の鏡面光成分
        float materialShininess;	//材質の鏡面光の鋭さ

        //vbo-球
        Vertex[] vertices2;         //頂点
        int[] indices2;             //頂点の指標（InitSphere内で頂点を指定している）
        int vbo2;                   //VBOのバッファの識別番号を保持
        int ibo2;                   //IBOのバッファの識別番号を保持
        int vao2;					//VAOの識別番号を保持
        int ColorTexture;                //背景画像
        int size = 128;             //textureサイズ

        //試験用
        int DepthTexture;
        //fbo
        int fbo_screen;

        #region shader
        int vertexShader;
        int fragmentShader;
        int shaderProgram;

        int mapLoc;
        int shyLoc;
        int clmLoc;
        //球面調和関数 Y
        float[] shy = new float[9] { 0.282095f, 0.488603f, 0.488603f, 0.488603f, 1.092548f, 1.092548f, 0.315392f, 1.092548f, 0.546274f };
        #endregion

        public Game() : base(800, 600, GraphicsMode.Default, "GraphicsWindow")
        {
            #region Camera__Initialize

            isCameraRotating = false;
            current = OpenTK.Vector2.Zero;
            previous = OpenTK.Vector2.Zero;
            rotate = Matrix4.Identity;
            zoom = 1.0f;
            wheelPrevious = 0.0f;
            #endregion

            #region Camera__Event

            //マウスボタンが押されると発生するイベント
            this.MouseDown += (sender, e) =>
            {
                var mouse = OpenTK.Input.Mouse.GetState();
                //右ボタンが押された場合
                if (e.Button == MouseButton.Right)
                {
                    //ok
                    isCameraRotating = true;
                    current = new OpenTK.Vector2(mouse.X, mouse.Y);
                }
            };

            //マウスボタンが離されると発生するイベント
            this.MouseUp += (sender, e) =>
            {
                //右ボタンが押された場合
                if (e.Button == MouseButton.Right)
                {
                    isCameraRotating = false;
                    previous = OpenTK.Vector2.Zero;
                }
            };

            //マウスが動くと発生するイベント
            this.MouseMove += (sender, e) =>
            {
                ////カメラが回転状態の場合
                if (isCameraRotating)
                {
                    var mouse = OpenTK.Input.Mouse.GetState();
                    previous = current;
                    current = new OpenTK.Vector2(mouse.X, mouse.Y);
                    OpenTK.Vector2 delta = current - previous;
                    delta /= (float)Math.Sqrt(this.Width * this.Width + this.Height * this.Height);
                    float length = delta.Length;
                    if (length > 0.0)
                    {
                        float rad = length * MathHelper.Pi;
                        /*
                        float theta = (float)Math.Sin(rad) / length;
                        */
                        OpenTK.Vector3 after = new OpenTK.Vector3(
                            delta.Y,
                            delta.X,
                            0.0f);
                        Matrix4 diff = Matrix4.CreateFromAxisAngle(after, rad);
                        Matrix4.Mult(ref rotate, ref diff, out rotate);
                    }
                }
            };

            //マウスホイールが回転すると発生するイベント
            this.MouseWheel += (sender, e) =>
            {
                var mouse = OpenTK.Input.Mouse.GetState();
                float delta = (float)mouse.Wheel - (float)wheelPrevious;
                zoom *= (float)Math.Pow(1.06, delta);
                //拡大、縮小の制限
                if (zoom > 2.0f)
                    zoom = 2.0f;
                if (zoom < 0.5f)
                    zoom = 0.5f;
                wheelPrevious = mouse.Wheel;
            };

            #endregion

            materialAmbient = new Color4(0.2f, 0.2f, 0.2f, 1.0f);
            materialDiffuse = new Color4(0.7f, 0.7f, 0.7f, 1.0f);
            materialSpecular = new Color4(0.6f, 0.6f, 0.6f, 1.0f);
            materialShininess = 80.0f;

            vbo2 = 0;
            ibo2 = 0;
            vao2 = 0;
            InitSphere(64, 32, 1.0f);//（縦の分割数⇒正面からは半分の面が見える,横の分割数,半径）

            VSync = VSyncMode.On;
        }

        //ウィンドウの起動時実行
        protected override void OnLoad(EventArgs e)
        {
            base.OnLoad(e);
            //ウィンドウの背景
            GL.ClearColor(0.3f, 0.5f, 0.8f, 0.0f);

            //Enable 使用可能にする（デプスバッファの使用）
            GL.Enable(EnableCap.DepthTest);
            GL.Enable(EnableCap.Normalize);
            GL.Enable(EnableCap.CullFace);//カリングの許可
            GL.Enable(EnableCap.Multisample);
            GL.Enable(EnableCap.VertexProgramPointSize);//ポイントの設定
            GL.Enable(EnableCap.PointSprite);
            GL.Enable(EnableCap.Blend); // ブレンドの許可

            GL.Disable(EnableCap.Lighting);

            //テクスチャの許可(2D・3D)
            GL.Enable(EnableCap.Texture2D);

            /*
            //裏面削除、反時計回りが表でカリング
            GL.Enable(EnableCap.CullFace);　//カリングの許可
            GL.CullFace(CullFaceMode.Back); //どちらの面を描画しないか
            GL.FrontFace(FrontFaceDirection.Ccw); //表を時計回り(Cw)か反時計回り(Ccw)か
            */

            #region texture
            //テクスチャ用のバッファを生成
            GL.GenTextures(1, out ColorTexture);
            GL.BindTexture(TextureTarget.Texture2D, ColorTexture);
            /*
            Bitmap file = new Bitmap("test1.png");
            //png画像の反転を直す
            file.RotateFlip(RotateFlipType.RotateNoneFlipY);
            //データ読み込み
            BitmapData data = file.LockBits(new Rectangle(0, 0, file.Width, file.Height), ImageLockMode.ReadOnly, System.Drawing.Imaging.PixelFormat.Format32bppArgb);
            GL.TexImage2D(TextureTarget.Texture2D, 0, PixelInternalFormat.Rgba, data.Width, data.Height, 0, OpenTK.Graphics.OpenGL.PixelFormat.Bgra, PixelType.UnsignedByte, data.Scan0);
            */
            //fboの時
            GL.TexImage2D(TextureTarget.Texture2D, 0, PixelInternalFormat.Rgba8, size, size, 0, OpenTK.Graphics.OpenGL.PixelFormat.Rgba, PixelType.UnsignedByte, IntPtr.Zero);
            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, (int)TextureMinFilter.Linear);
            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, (int)TextureMagFilter.Linear);
            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureWrapS, (int)TextureWrapMode.MirroredRepeat);
            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureWrapT, (int)TextureWrapMode.MirroredRepeat);

            //GL.BindTexture(TextureTarget.Texture2D, 0);
            #endregion

            //各Arrayを有効化
            GL.EnableClientState(ArrayCap.VertexArray);
            GL.EnableClientState(ArrayCap.NormalArray);
            GL.EnableClientState(ArrayCap.TextureCoordArray);
            GL.EnableClientState(ArrayCap.ColorArray);

            #region vbo
            //VBOを1コ生成し、2の頂点データを送り込む
            GL.GenBuffers(1, out vbo2);
            //ArrayBufferとしてvbo2を指定（バインド）
            GL.BindBuffer(BufferTarget.ArrayBuffer, vbo2);
            int vertexArray2Size = vertices2.Length * Vertex.Size;
            //ArrayBufferにデータをセット
            GL.BufferData<Vertex>(BufferTarget.ArrayBuffer, new IntPtr(vertexArray2Size), vertices2, BufferUsageHint.StaticDraw);
            //バインド解除
            GL.BindBuffer(BufferTarget.ArrayBuffer, 0);
            #endregion

            #region ibo
            GL.GenBuffers(1, out ibo2);
            GL.BindBuffer(BufferTarget.ElementArrayBuffer, ibo2);
            int indexArray2Size = indices2.Length * sizeof(int);
            GL.BufferData(BufferTarget.ElementArrayBuffer, new IntPtr(indexArray2Size), indices2, BufferUsageHint.StaticDraw);
            GL.BindBuffer(BufferTarget.ElementArrayBuffer, 0);
            #endregion

            #region vao
            //VAOを1コ作成
            GL.GenVertexArrays(1, out vao2);
            GL.BindVertexArray(vao2);
            //各Arrayを有効化
            GL.EnableClientState(ArrayCap.VertexArray);
            GL.EnableClientState(ArrayCap.NormalArray);
            GL.EnableClientState(ArrayCap.TextureCoordArray);
            GL.EnableClientState(ArrayCap.ColorArray);

            GL.BindBuffer(BufferTarget.ArrayBuffer, vbo2);
            //頂点の位置、法線、テクスチャ情報の場所を指定
            GL.VertexPointer(3, VertexPointerType.Float, Vertex.Size, 0);
            GL.NormalPointer(NormalPointerType.Float, Vertex.Size, OpenTK.Vector3.SizeInBytes);
            GL.TexCoordPointer(2, TexCoordPointerType.Float, Vertex.Size, OpenTK.Vector3.SizeInBytes * 2);
            GL.ColorPointer(4, ColorPointerType.Float, Vertex.Size, OpenTK.Vector3.SizeInBytes * 2 + OpenTK.Vector2.SizeInBytes);
            GL.BindBuffer(BufferTarget.ArrayBuffer, 0);

            GL.BindVertexArray(0);
            #endregion()

            #region Shader
            int status;
            //バーテックスシェーダを生成
            vertexShader = GL.CreateShader(ShaderType.VertexShader);
            using (var sr = new StreamReader("shader.vert"))
            {
                //バーテックスシェーダのコードを指定
                GL.ShaderSource(vertexShader, sr.ReadToEnd());
            }
            //バーテックスシェーダをコンパイル
            GL.CompileShader(vertexShader);
            GL.GetShader(vertexShader, ShaderParameter.CompileStatus, out status);
            //コンパイル結果をチェック
            if (status == 0)
            {
                throw new ApplicationException(GL.GetShaderInfoLog(vertexShader));
            }

            //シェーダオブジェクト(フラグメント)を生成
            fragmentShader = GL.CreateShader(ShaderType.FragmentShader);

            using (var sr = new StreamReader("shader.frag"))
            {
                //フラグメントシェーダのコードを指定
                GL.ShaderSource(fragmentShader, sr.ReadToEnd());
            }
            //フラグメントシェーダをコンパイル
            GL.CompileShader(fragmentShader);
            GL.GetShader(fragmentShader, ShaderParameter.CompileStatus, out status);
            //コンパイル結果をチェック
            if (status == 0)
            {
                throw new ApplicationException(GL.GetShaderInfoLog(fragmentShader));
            }

            //シェーダプログラムの生成
            shaderProgram = GL.CreateProgram();
            //各シェーダオブジェクトをシェーダプログラムへ登録
            GL.AttachShader(shaderProgram, vertexShader);
            GL.AttachShader(shaderProgram, fragmentShader);
            //不要になったシェーダオブジェクトの削除
            GL.DeleteShader(vertexShader);
            GL.DeleteShader(fragmentShader);
            //シェーダプログラムのリンク
            GL.LinkProgram(shaderProgram);
            //GL.GetProgram(shaderProgram, ProgramParameter.LinkStatus, out status);
            GL.GetProgram(shaderProgram, GetProgramParameterName.LinkStatus, out status);
            //シェーダプログラムのリンクのチェック
            if (status == 0)
            {
                throw new ApplicationException(GL.GetProgramInfoLog(shaderProgram));
            }
            mapLoc = GL.GetUniformLocation(shaderProgram, "irradiance");
            shyLoc = GL.GetUniformLocation(shaderProgram, "shy");
            clmLoc = GL.GetUniformLocation(shaderProgram, "clm");
            GL.Uniform1(shyLoc, 9, shy);
            //シェーダプログラムを使用
            GL.UseProgram(shaderProgram);
            #endregion

            #region fbo
            //一応Depthも試験的に作ってみる
            GL.GenTextures(1, out DepthTexture);
            GL.BindTexture(TextureTarget.Texture2D, DepthTexture);
            GL.TexImage2D(TextureTarget.Texture2D, 0, (PixelInternalFormat)All.DepthComponent32, size, size, 0, OpenTK.Graphics.OpenGL.PixelFormat.DepthComponent, PixelType.UnsignedInt, IntPtr.Zero);
            // things go horribly wrong if DepthComponent's Bitcount does not match the main Framebuffer's Depth
            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, (int)TextureMinFilter.Linear);
            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, (int)TextureMagFilter.Linear);
            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureWrapS, (int)TextureWrapMode.ClampToBorder);
            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureWrapT, (int)TextureWrapMode.ClampToBorder);
            GL.BindTexture(TextureTarget.Texture2D, 0);
            //FBO============================================
            GL.Ext.GenFramebuffers(1, out fbo_screen);
            GL.Ext.BindFramebuffer(FramebufferTarget.FramebufferExt, fbo_screen);
            GL.Ext.FramebufferTexture2D(FramebufferTarget.FramebufferExt, FramebufferAttachment.ColorAttachment0Ext, TextureTarget.Texture2D, ColorTexture, 0);
            GL.Ext.FramebufferTexture2D(FramebufferTarget.FramebufferExt, FramebufferAttachment.DepthAttachmentExt, TextureTarget.Texture2D, DepthTexture, 0);
            //エラーチェック
            FramebufferErrorCode fbostatus = GL.CheckFramebufferStatus(FramebufferTarget.FramebufferExt);
            if (fbostatus != FramebufferErrorCode.FramebufferComplete &&
                fbostatus != FramebufferErrorCode.FramebufferCompleteExt)
            {
                Console.WriteLine("Error creating framebuffer: {0}", status);
            }
            //FBO（https://github.com/mono/opentk/blob/main/Source/Examples/OpenGL/1.x/FramebufferObject.cs）
            GL.PushAttrib(AttribMask.ViewportBit);
            {
                GL.ClearColor(0f, 0f, 0f, 0f);
                GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

                //描画サイズ・カメラの設定
                GL.Viewport(0, 0, size, size);
                OpenTK.Matrix4 perspective = OpenTK.Matrix4.CreateOrthographic(size, size, 0f, 1.0f);
                GL.MatrixMode(MatrixMode.Projection);
                GL.LoadMatrix(ref perspective);

                Matrix4 lookat = Matrix4.LookAt(0f, 0f, 1.0f, 0f, 0f, 0f, 0f, 1f, 0f);
                GL.MatrixMode(MatrixMode.Modelview);
                GL.LoadMatrix(ref lookat);

                //FBOの加算を有効に
                GL.Enable(EnableCap.Blend);
                GL.BlendFunc(BlendingFactor.OneMinusDstColor, BlendingFactor.One);
                //GL.BlendEquation(BlendEquationMode.FuncAdd);

                //http://penguinitis.g1.xrea.com/computer/programming/OpenGL/23-blend.html
                GL.Disable(EnableCap.DepthTest); //z座標が同じものをBlendするにはDepthTestを切る

                //(128, 256, 0)
                //MakeMap(128f, 256f, 0.0f);
                MakeMap_SH();


                GL.Disable(EnableCap.Blend);
                GL.Enable(EnableCap.DepthTest);
                GL.UseProgram(0);
            }
            GL.PopAttrib();
            GL.Ext.BindFramebuffer(FramebufferTarget.FramebufferExt, 0); // disable rendering into the FBO

            //初期画面背景
            GL.ClearColor(Color4.DarkBlue);
            #endregion

        }

        //ウィンドウの終了時に実行される。
        protected override void OnUnload(EventArgs e)
        {
            base.OnUnload(e);

            GL.DeleteBuffers(1, ref vbo2);          //バッファを1コ削除

            GL.DeleteBuffers(1, ref vbo2);          //バッファを1コ削除
            GL.DeleteBuffers(1, ref ibo2);          //バッファを1コ削除
            GL.DeleteVertexArrays(1, ref vao2);     //VAOを1コ削除

            GL.DisableClientState(ArrayCap.VertexArray);    //VertexArrayを無効化
            GL.DisableClientState(ArrayCap.NormalArray);    //NormalArrayを無効化
            GL.DisableClientState(ArrayCap.ColorArray);		//ColorArrayを無効化
            GL.DisableClientState(ArrayCap.TextureCoordArray);

            GL.DeleteTexture(ColorTexture);   //使用したテクスチャの削除

            GL.DeleteProgram(shaderProgram);//シェーダの削除

            GL.DeleteFramebuffers(1, ref fbo_screen);
        }

        //ウィンドウサイズが変更された時に実行
        protected override void OnResize(EventArgs e)
        {
            base.OnResize(e);

            GL.Viewport(ClientRectangle);
        }

        //画面更新で実行される。
        protected override void OnUpdateFrame(FrameEventArgs e)
        {
            base.OnUpdateFrame(e);
            KeyboardState input = Keyboard.GetState();
            if (input.IsKeyDown(Key.Escape))
            {
                Close();
            }

            #region Camera__Keyboard

            //F1キーで回転をリセット
            if (input.IsKeyDown(Key.F1))
            {
                rotate = Matrix4.Identity;
            }

            //F2キーでY軸90度回転
            if (input.IsKeyDown(Key.F2))
            {
                rotate = Matrix4.CreateRotationY(MathHelper.PiOver2);
            }

            //F3キーでY軸180度回転
            if (input.IsKeyDown(Key.F3))
            {
                rotate = Matrix4.CreateRotationY(MathHelper.Pi);
            }

            //F4キーでY軸270度回転
            if (input.IsKeyDown(Key.F4))
            {
                rotate = Matrix4.CreateRotationY(MathHelper.ThreePiOver2);
            }

            //F5キーで拡大をリセット
            if (input.IsKeyDown(Key.F5))
            {
                zoom = 1.0f;
            }

            #endregion
        }

        //画面描画で実行される。
        protected override void OnRenderFrame(FrameEventArgs e)
        {
            base.OnRenderFrame(e);
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);


            #region TransFormationMatrix

            Matrix4 modelView = Matrix4.LookAt(OpenTK.Vector3.UnitZ * 10 / zoom, OpenTK.Vector3.Zero, OpenTK.Vector3.UnitY);
            GL.MatrixMode(MatrixMode.Modelview);
            GL.LoadMatrix(ref modelView);
            GL.MultMatrix(ref rotate);

            Matrix4 projection = Matrix4.CreatePerspectiveFieldOfView(MathHelper.PiOver4 / zoom, (float)this.Width / (float)this.Height, 1.0f, 64.0f);
            GL.MatrixMode(MatrixMode.Projection);
            GL.LoadMatrix(ref projection);

            #endregion

            //材質のパラメータ設定（表裏、材質の要素、その情報）
            GL.Material(MaterialFace.Front, MaterialParameter.Ambient, materialAmbient);
            GL.Material(MaterialFace.Front, MaterialParameter.Diffuse, materialDiffuse);
            GL.Material(MaterialFace.Front, MaterialParameter.Specular, materialSpecular);
            GL.Material(MaterialFace.Front, MaterialParameter.Shininess, materialShininess);

            GL.Viewport(0, 0, Width, Height);

            GL.BindTexture(TextureTarget.Texture2D, ColorTexture);

            //球を描画
            GL.BindVertexArray(vao2);
            GL.BindBuffer(BufferTarget.ElementArrayBuffer, ibo2);
            GL.DrawElements(BeginMode.Quads, indices2.Length, DrawElementsType.UnsignedInt, 0);
            GL.BindVertexArray(0);


            //Texture
            GL.Color4(Color4.White);
            GL.Begin(BeginMode.Quads);

            GL.TexCoord2(1.0, 1.0);
            GL.Vertex3(3, 1, 0);

            GL.TexCoord2(0.0, 1.0);
            GL.Vertex3(1, 1, 0);

            GL.TexCoord2(0.0, 0.0);
            GL.Vertex3(1, -1, 0);

            GL.TexCoord2(1.0, 0.0);
            GL.Vertex3(3, -1, 0);

            GL.End();

            GL.BindTexture(TextureTarget.Texture2D, 0);

            SwapBuffers();
        }

        //球で初期化
        void InitSphere(int slice, int stack, float radius)
        {
            LinkedList<Vertex> vertexList = new LinkedList<Vertex>();
            LinkedList<int> indexList = new LinkedList<int>();

            for (int i = 0; i <= stack; i++)
            {
                double p = Math.PI / stack * i;
                double pHeight = Math.Cos(p);
                double pWidth = Math.Sin(p);

                for (int j = 0; j <= slice; j++)
                {
                    double rotor = 2 * Math.PI / slice * j;
                    double x = Math.Cos(rotor);
                    double y = Math.Sin(rotor);

                    OpenTK.Vector3 position = new OpenTK.Vector3((float)(radius * x * pWidth), (float)(radius * pHeight), (float)(radius * y * pWidth));
                    OpenTK.Vector3 normal = new OpenTK.Vector3((float)(x * pWidth), (float)pHeight, (float)(y * pWidth));
                    OpenTK.Vector2 uv = new OpenTK.Vector2((float)((1 + x * pWidth) / 2), (float)((1 + pHeight) / 2));
                    OpenTK.Vector4 color = new OpenTK.Vector4(1.0f, 0.0f, 0.0f, 1.0f);
                    vertexList.AddLast(new Vertex(position, normal, uv, color));
                }
            }

            for (int i = 0; i <= stack; i++)
            {
                for (int j = 0; j <= slice; j++)
                {
                    int d = i * (slice + 1) + j;
                    indexList.AddLast(d);
                    indexList.AddLast(d + 1);
                    indexList.AddLast(d + slice + 2);
                    indexList.AddLast(d + slice + 1);
                }
            }
            vertices2 = vertexList.ToArray();
            indices2 = indexList.ToArray();
        }

        /*
        //トーラスの初期化
        void InitTorus(int row, int column, double smallRadius, double largeRadius)
        {
            LinkedList<Vertex> vertexList = new LinkedList<Vertex>();
            LinkedList<int> indexList = new LinkedList<int>();
            for (int i = 0; i <= row; i++)
            {
                double sr = (2.0 * Math.PI / row) * i;
                double cossr = Math.Cos(sr);
                double sinsr = Math.Sin(sr);
                double sx = cossr * smallRadius;
                double sy = sinsr * smallRadius;
                for (int j = 0; j <= column; j++)
                {
                    double lr = (2.0 * Math.PI / column) * j;
                    double coslr = Math.Cos(lr);
                    double sinlr = Math.Sin(lr);
                    double px = coslr * (sx + largeRadius);
                    double py = sy;
                    double pz = sinlr * (sx + largeRadius);
                    double nx = cossr * coslr;
                    double ny = sinsr;
                    double nz = cossr * sinlr;
                    OpenTK.Vector3 position = new OpenTK.Vector3((float)px, (float)py, (float)pz);
                    OpenTK.Vector3 normal = new OpenTK.Vector3((float)nx, (float)ny, (float)nz);
                    OpenTK.Vector2 uv = new OpenTK.Vector2((float)(nx + 1) / 2, (float)(ny + 1) / 2);
                    vertexList.AddLast(new Vertex(position, normal, uv));
                }
            }
            for (int i = 0; i < row; i++)
            {
                for (int j = 0; j < column; j++)
                {
                    int d = i * (column + 1) + j;
                    indexList.AddLast(d);
                    indexList.AddLast(d + column + 1);
                    indexList.AddLast(d + column + 2);
                    indexList.AddLast(d + 1);
                }
            }
            vertices1 = vertexList.ToArray();
            indices1 = indexList.ToArray();
        }
        */

        //球を描画する
        private void DrawSphere()
        {
            int slices = 24, stacks = 24;   //横と縦の分割数
            double r = 1.24;                //半径
            for (int i = 0; i < stacks; i++)
            {
                //輪切り上部
                double upper = Math.PI / stacks * i;
                double upperHeight = Math.Cos(upper);
                double upperWidth = Math.Sin(upper);
                //輪切り下部
                double lower = Math.PI / stacks * (i + 1);
                double lowerHeight = Math.Cos(lower);
                double lowerWidth = Math.Sin(lower);

                GL.Begin(BeginMode.QuadStrip);
                for (int j = 0; j <= slices; j++)
                {
                    //輪切りの面を単位円としたときの座標
                    double rotor = 2 * Math.PI / slices * j;
                    double x = Math.Cos(rotor);
                    double y = Math.Sin(rotor);
                    GL.Color4(1.0f, 0.0f, 0.0f, 1.0f);
                    GL.Normal3(x * lowerWidth, lowerHeight, y * lowerWidth);
                    GL.TexCoord2(0.5f + x * lowerWidth / 2, lowerHeight / 2 + 0.5f);
                    GL.Vertex3(r * x * lowerWidth, r * lowerHeight, r * y * lowerWidth);
                    GL.Normal3(x * upperWidth, upperHeight, y * upperWidth);
                    GL.TexCoord2(x * upperWidth / 2 + 0.5f, upperHeight / 2 + 0.5f);
                    GL.Vertex3(r * x * upperWidth, r * upperHeight, r * y * upperWidth);
                }
                GL.End();
            }
        }

        //ポイントスプライトで一つの円を描こう
        void MakeMap_SH()
        {
            //マップのUV座標
            int mu = 0, mv = 0;
            int nx = 0, ny = 0;
            float half_w = MainWindow.map_w / 2;
            float half_h = MainWindow.map_h / 2;
            float x = 0, y = 0, z = 0;
            //球面調和関数展開
            float[] clm = new float[9];
            //clmを求めていく
            for (int i = 0; i < 9; i++)
            {
                float sum = 0;
                float n_data = 0;
                for (int j = 0; j < MainWindow.irradiancemap.GetLength(0); j++)
                {
                    if (MainWindow.irradiancemap[j, 0] != 0 && MainWindow.irradiancemap[j, 1] != 0)
                    {
                        mu = j % MainWindow.map_w;
                        mv = j / MainWindow.map_w;
                        x = (mu - half_w) / half_w;
                        y = (half_h - mv) / half_h;
                        float sin = (float)Math.Sqrt(x * x + y * y);
                        if(sin > 1)
                        {
                            z = 0;
                        }
                        else
                        {
                            z = (float)Math.Sqrt(1 - x * x - y * y);
                        }

                        if (i == 0)
                        {
                            sum += (float)(MainWindow.irradiancemap[j, 0] + MainWindow.irradiancemap[j, 1]) * shy[i] * sin;
                        }else if (i == 8)//Y22
                        {
                            sum += (float)(MainWindow.irradiancemap[j, 0] + MainWindow.irradiancemap[j, 1]) * shy[i] * (x * x - y * y) * sin;
                        }else if (i == 6)
                        {
                            sum += (float)(MainWindow.irradiancemap[j, 0] + MainWindow.irradiancemap[j, 1]) * shy[i] * (3 * z * z - 1) * sin;
                        }else if (i == 1)
                        {
                            sum += (float)(MainWindow.irradiancemap[j, 0] + MainWindow.irradiancemap[j, 1]) * shy[i] * y * sin;
                        }else if (i == 2)
                        {
                            sum += (float)(MainWindow.irradiancemap[j, 0] + MainWindow.irradiancemap[j, 1]) * shy[i] * z * sin;
                        }else if (i == 3)
                        {
                            sum += (float)(MainWindow.irradiancemap[j, 0] + MainWindow.irradiancemap[j, 1]) * shy[i] * x * sin;
                        }else if (i == 4)
                        {
                            sum += (float)(MainWindow.irradiancemap[j, 0] + MainWindow.irradiancemap[j, 1]) * shy[i] * x * y * sin;
                        }
                        else if (i == 5)
                        {
                            sum += (float)(MainWindow.irradiancemap[j, 0] + MainWindow.irradiancemap[j, 1]) * shy[i] * z * y * sin;
                        }
                        else if (i == 7)
                        {
                            sum += (float)(MainWindow.irradiancemap[j, 0] + MainWindow.irradiancemap[j, 1]) * shy[i] * x * z * sin;
                        }
                        n_data++;

                    }
                }
                //これでC_l^mは求まった
                clm[i] = sum / n_data;
                Debug.WriteLine(clm[i]);
            }
            GL.Uniform1(clmLoc, 9, clm);

            //同じ座標に点が重ねられていて真っ白っぽくなっている（たぶん投影変換のところ）(座標は256✖256)
            GL.PointSize(128);
            GL.Begin(BeginMode.Points);
            GL.Color4(1f, 1f, 11f, 1.0f);
            GL.Vertex3(0,0,0);
            GL.End();
        }

        float Clm_compute(int l, int m, int theta, int phi)
        {

            return 0;
        }

        //放射照度マップの描画(w：描画範囲の縦横幅)(各点をポイントスプライトで重ねる➡密度に依存する)
        void MakeMap(float wh, float r, float depth)
        {
            //マップのUV座標
            int mu = 0, mv = 0;
            int nx = 0, ny = 0;
            float half_w = MainWindow.map_w / 2;
            float half_h = MainWindow.map_h / 2;
            
            //マップ上に点を打つ

            //同じ座標に点が重ねられていて真っ白っぽくなっている（たぶん投影変換のところ）(座標は256✖256)
            GL.PointSize(r);
            GL.Begin(BeginMode.Points);
            GL.Color4(0.0f, 1.0f, 0.1f, 1.0f);
            GL.Vertex3(128.0f, 0.0f, depth);
            GL.End();
            float[] elem = new float[2];
            for (int i = 0; i < MainWindow.irradiancemap.GetLength(0); i++)
            {
                if (MainWindow.irradiancemap[i, 0] != 0 && MainWindow.irradiancemap[i, 1] != 0)
                {
                    elem[0] = (float)MainWindow.irradiancemap[i, 0];
                    elem[1] = (float)MainWindow.irradiancemap[i, 1];
                    GL.Uniform2(mapLoc, 1, elem);
                    //座標は０スタート
                    mu = i % MainWindow.map_w;
                    mv = i / MainWindow.map_w;
                    //uv座標を-1～1の範囲に変換（-wh～wh）
                    GL.PointSize(r);
                    GL.Begin(BeginMode.Points);
                    GL.Color4(0.1f, 0.1f, 0.1f, 1.0f);
                    GL.Vertex3((float)(mu - half_w) / half_w * wh, (float)(half_h - mv) / half_h * wh, depth);
                    GL.End();
                }
            }
            //正規化

        }
    }
}
