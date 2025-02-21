// Seccion de importe de librerias
#include <rclcpp/rclcpp.hpp>
// Importe de librerias de interfaz de lectura
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
// Importe de libreria de opencv
#include <opencv2/opencv.hpp>
// Seccion de importe de librerias utilitarias
#include <chrono>
#include <cv_bridge/cv_bridge.h>

// Definicion de namespace de funciones especificas
using std::placeholders::_1;

// Crecion de calse de nodo de ejecucion de republicador
class ImgRePublisher : public rclcpp::Node
{
    public:
        // Constructor de nodo
        ImgRePublisher() : Node("Image_node_republisher")
        {
            // Declaracion de parametros de nodo
            this->declare_parameter<int>("image_reliability", 1);

            // Designacion de valor de parametro
            int my_param = this->get_parameter("image_reliability").as_int();


            // Declaracion de perfil de qos
            rclcpp::QoS image_qos_profile(
                rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)
            );
            // Configuracion de protocolo de recepcion de datos
            if (my_param == 1){
              image_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
            } else{
              image_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
            };
            image_qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
            image_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
            image_qos_profile.keep_last(1);

            // Creacion de publicador
            img_p_ = this->create_publisher<sensor_msgs::msg::Image>(
                "/ntarobot/img",
                image_qos_profile
            );

            // Utilizacion de image subscription
            comp_img_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                // "/camera/image_raw/compressed",
                "/color/image_raw/compressed",
                image_qos_profile,
                std::bind(&ImgRePublisher::comp_img_callback, this, _1)
            );

            
        }

    private:
        // Definicion de variables
        // Definicion de subscriptores
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr comp_img_;
        // Creacion de publicadores
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_p_;
        // Definicion de funciones generales
        void comp_img_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
        
};

// Funcion de procesamiento de imagen recibida
void ImgRePublisher::comp_img_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // Catch error
    try{
        // Conversion de mensaje a matriz de opencv
        cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

        // Condicional de validacion de existencia de imagen
        if (image.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "Error al decodificar la imagen.");
                return;
            }

        // Convertir la imagen OpenCV a un mensaje ROS Image
        std_msgs::msg::Header header;
        // Establecer el timestamp actual
        header.stamp = this->get_clock()->now();  
        cv_bridge::CvImage cv_image(header, "bgr8", image);
        sensor_msgs::msg::Image::SharedPtr img_msg = cv_image.toImageMsg();
        // Publicar la imagen decodificada
        img_p_->publish(*img_msg);
        // Validacion por mensaje de republicacion de iamgen
        // RCLCPP_INFO(this->get_logger(), "Imagen recibida, decodificada y publicada.");
    }
    catch (cv::Exception &e)
    {
        // En caso exista un erro se publicara dicho error
        RCLCPP_ERROR(this->get_logger(), "Error al procesar la imagen: %s", e.what());
    }
}

// Funcion main de ejecucion
int main(int argc, char* argv[]){
    // Inicializacion de nodo
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImgRePublisher>();

    // Ejecucion de uso de nodo
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    // Retorno de seguranza
    return 0;
}


