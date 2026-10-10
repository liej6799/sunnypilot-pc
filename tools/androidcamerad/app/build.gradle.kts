plugins {
  id("com.android.application")
}

android {
  namespace = "ai.sunnypilot.camerabridge"
  compileSdk = 34
  ndkVersion = "25.1.8937393"

  defaultConfig {
    applicationId = "ai.sunnypilot.camerabridge"
    minSdk = 26
    targetSdk = 34
    versionCode = 1
    versionName = "0.1.0"

    externalNativeBuild {
      cmake {
        cppFlags += "-std=c++20"
      }
    }
    ndk {
      abiFilters += "arm64-v8a"
    }
  }

  externalNativeBuild {
    cmake {
      path = file("src/main/cpp/CMakeLists.txt")
      version = "3.22.1"
    }
  }

  compileOptions {
    sourceCompatibility = JavaVersion.VERSION_17
    targetCompatibility = JavaVersion.VERSION_17
  }

  packaging {
    jniLibs.useLegacyPackaging = false
  }
}
