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

            if (click == true)
            {
                //カラー情報とデプス情報を別で保存して、メモリ解放する
                click = false;
                calibrationData = mapper.GetDepthCameraIntrinsics();

                //(1)Depthの平滑化
                //Depthが取得できず、周りの情報も不足している場所の保存
                /*
                for (int i = 0; i < this.depthFrameData.Length; ++i)
                {
                    if (depthFrameData[i] == 0 && i + depthFrameDescription.Width < depthFrameData.Length && i - depthFrameDescription.Width > 0)
                    {
                        depthFrameData[i] = (ushort)((depthFrameData[i + 1] + depthFrameData[i - 1] + depthFrameData[i + depthFrameDescription.Width] + depthFrameData[i - depthFrameDescription.Width]) / 4);
                    }
                }*/
                // BitmapSourceを保存する
                /*
                using (Stream stream = new FileStream("test.png", FileMode.Create))
                {
                    PngBitmapEncoder encoder = new PngBitmapEncoder();
                    encoder.Frames.Add(BitmapFrame.Create(depth));
                    encoder.Save(stream);
                }*/
                Mat src = BitmapSourceConverter.ToMat(depth);
                Kmeans_segmentation(colorFrameData);

                //頂点マップの作成
                var vertexData = new int[depthFrameDescription.LengthInPixels * colorFrameDescription.BytesPerPixel];
                //var normalData = new Vector3[depthFrameDescription.LengthInPixels];
                double[,] normalData = new double[depthFrameDescription.LengthInPixels, 3];
                depthFrameData = BilateralFilter(depthFrameData);
                vertexData = VertexmapCreate(depthFrameData);
                normalData = NormalmapCreate(vertexData);
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

        /*==========================頂点マップから法線マップを作成する関数(もう使わない)=======================================*/
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

        /*=========================================頂点マップから法線マップを作成する関数(Holzerらの手法)====================================================================*/
        private double[,] NormalmapCreate(int[] VertexData)
        {
            //x方向:vx , y方向:vy , 法線方向:normalData
            double[,] normalData = new double[depthFrameDescription.LengthInPixels, 3];
            double[] vx = new double[3];
            double[] vy = new double[3];
            double[] n = new double[3];
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

                n = VecNormalized(VecCross(vx, vy));
                normalData[i, 0] = n[0];
                normalData[i, 1] = n[1];
                normalData[i, 2] = n[2];
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
        /*=========================================↑テスト=================================================================*/

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

        private void Kmeans_segmentation(byte[] colorbuffer)
        {
            //Cv2.Kmeans;
            const int CLASS = 16;
            using (Mat src = new Mat(depthFrameDescription.Width * depthFrameDescription.Height, 1, MatType.CV_32FC3))
            {
                using (Mat cluster = new Mat())
                {
                    using (Mat center = new Mat(CLASS, 1, MatType.CV_32FC3))
                    {
                        int i = 0;
                        long index = 0;
                        //引数byte[]をKmeans()に適した形にする
                        for(int y = 0; y < depthFrameDescription.Height; y++)
                        {
                            for (int x = 0; x < depthFrameDescription.Width; x++ ,i++){
                                index = i * colorFrameDescription.BytesPerPixel;
                                Vec3f vec3f = new Vec3f
                                {
                                    Item0 = colorbuffer[index],
                                    Item1 = colorbuffer[index + 1],
                                    Item2 = colorbuffer[index + 2]
                                };
                                src.Set<Vec3f>(i, vec3f);
                            }
                        }
                        var criteria = new TermCriteria(type: CriteriaType.Eps | CriteriaType.MaxIter, maxCount: 10, epsilon: 1.0);
                        Cv2.Kmeans(src, CLASS, cluster, criteria, 3, KMeansFlags.PpCenters, center);
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
