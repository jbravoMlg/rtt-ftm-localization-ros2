plugins {
    alias(libs.plugins.androidApplication)
    alias(libs.plugins.jetbrainsKotlinAndroid)
}

val ros2CoreJars = listOf(
    "rcljava.jar",
    "rcljava_common.jar",
    "commons-lang3-3.7.jar",
    "slf4j-api-1.7.21.jar",
    "slf4j-android-1.7.21.jar",
    "std_msgs_messages.jar",
    "std_srvs_messages.jar",
    "builtin_interfaces_messages.jar",
    "rcl_interfaces_messages.jar",
    "rmw_dds_common_messages.jar",
    "rosgraph_msgs_messages.jar",
)

val unusedRosJniPatterns = setOf(
    "**/libbenchmark*.so",
    "**/libperformance_test_fixture.so",
    "**/libtest_msgs*.so",
    "**/libvisualization_msgs*.so",
    "**/libnav_msgs*.so",
    "**/liblifecycle_msgs*.so",
    "**/libtf2_msgs*.so",
    "**/libdiagnostic_msgs*.so",
    "**/libcomposition_interfaces*.so",
    "**/libtrajectory_msgs*.so",
    "**/libshape_msgs*.so",
    "**/libaction_msgs*.so",
    "**/libactionlib_msgs*.so",
    "**/libstatistics_msgs*.so",
    "**/libstereo_msgs*.so",
    "**/libexample_interfaces*.so",
    "**/libsensor_msgs*.so",
    "**/libgeometry_msgs*.so",
)

android {
    namespace = "com.jbravo.osa_ftm"
    compileSdk = 34

    defaultConfig {
        applicationId = "com.jbravo.osa_ftm"
        // Wi-Fi RTT requiere API 28+. Tu minSdk=30 está OK.
        minSdk = 30
        targetSdk = 34
        versionCode = 1
        versionName = "1.0"

        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
        vectorDrawables { useSupportLibrary = true }
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }
    kotlinOptions { jvmTarget = "1.8" }

    buildFeatures {
        // seguimos con Compose
        compose = true
        // viewBinding no se necesita ya
        // viewBinding = false
    }

    composeOptions {
        kotlinCompilerExtensionVersion = "1.5.1"
    }

    packaging {
        resources { excludes += "/META-INF/{AL2.0,LGPL2.1}" }
        jniLibs { excludes += unusedRosJniPatterns }
    }
}

dependencies {

    ros2CoreJars.forEach { implementation(files("libs/$it")) }
    implementation(fileTree(mapOf("dir" to "libs", "include" to listOf("*.aar"))))

    implementation(libs.androidx.core.ktx)
    implementation(libs.androidx.lifecycle.runtime.ktx)
    implementation(libs.androidx.activity.compose)
    implementation(libs.androidx.fragment.ktx)
    implementation(platform(libs.androidx.compose.bom))
    implementation(libs.androidx.ui)
    implementation(libs.androidx.ui.graphics)
    implementation(libs.androidx.ui.tooling.preview)
    implementation(libs.androidx.material3)

    implementation("com.github.mik3y:usb-serial-for-android:3.5.1")


    // ===== NUEVO: GNSS (Fused Location) =====
    implementation("com.google.android.gms:play-services-location:21.3.0")

    // ===== ROS 2 (ajusta a tu setup) =====
    // Si tienes AAR/JAR locales en app/libs:
    // implementation(fileTree(mapOf("dir" to "libs", "include" to listOf("*.jar","*.aar"))))
    // Y/o dependencias Maven si las tienes publicadas internamente:
    // implementation("org.ros2.rcljava:rcljava:<versión>")
    // implementation("org.ros2.rcljava:std_msgs:<versión>")



    // Si usas BOM:
    implementation("androidx.compose:compose-bom:2024.06.00")
    implementation("androidx.compose.material3:material3")
    implementation("androidx.compose.material:material-icons-extended")


    testImplementation(libs.junit)
    androidTestImplementation(libs.androidx.junit)
    androidTestImplementation(libs.androidx.espresso.core)
    androidTestImplementation(platform(libs.androidx.compose.bom))
    androidTestImplementation(libs.androidx.ui.test.junit4)
    debugImplementation(libs.androidx.ui.tooling)
    debugImplementation(libs.androidx.ui.test.manifest)
}
