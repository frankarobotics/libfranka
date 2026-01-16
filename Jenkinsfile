def PYTHON_VERSION_BY_UBUNTU = ['20.04': '3.9', '22.04': '3.10', '24.04': '3.12']

pipeline {
  libraries {
    lib('fe-pipeline-steps@1.0.0')
  }
  agent {
      node {
        label 'docker'
      }
  }
  triggers {
    pollSCM('H/5 * * * *')
  }
  options {
    parallelsAlwaysFailFast()
    timeout(time: 2, unit: 'HOURS')
  }
  environment {
    VERSION = feDetermineVersionFromGit()
    UNSTABLE = 'UNSTABLE'
  }
  stages {
    stage('Matrix') {
      matrix {
        axes {
          axis {
            name 'UBUNTU_VERSION'
            values '20.04', '22.04', '24.04'
          }
        }
        environment {
          DISTRO = "${['20.04': 'focal', '22.04': 'jammy', '24.04': 'noble'][UBUNTU_VERSION]}"
          PYTHON_VERSION = "${PYTHON_VERSION_BY_UBUNTU[UBUNTU_VERSION]}"
        }
        agent {
          dockerfile {
            dir ".ci"
            filename "Dockerfile"
            reuseNode false
            additionalBuildArgs "--pull --build-arg UBUNTU_VERSION=${env.UBUNTU_VERSION} --build-arg PYTHON_VERSION=${PYTHON_VERSION_BY_UBUNTU[env.UBUNTU_VERSION]} --tag libfranka:${env.UBUNTU_VERSION}"
            args '--privileged ' +
                 '--cap-add=SYS_PTRACE ' +
                 '--security-opt seccomp=unconfined ' +
                 '--shm-size=2g '
         }
        }
        stages {
          stage('Init Distro') {
            steps {
              script {
                if (!env.DISTRO) {
                  error "Unknown UBUNTU_VERSION=${env.UBUNTU_VERSION}"
                }
                if (!env.PYTHON_VERSION) {
                  error "Unknown PYTHON_VERSION for UBUNTU_VERSION=${env.UBUNTU_VERSION}"
                }
              }
            }
          }
          stage('Setup') {
            stages {
              stage('Notify Stash') {
                steps {
                  script {
                    notifyBitbucket()
                  }
                }
              }
              stage('Clean Workspace') {
                steps {
                  script {
                    def distro = DISTRO_VERSIONS[env.UBUNTU_VERSION]
                    withEnv(["DISTRO=${distro}"]) {
                      sh '''
                        # Clean build dirs for this distro axis
                        rm -rf build-*${DISTRO}
                        rm -rf install-*${DISTRO}
                        # Remove corrupted googletest fetch content if present
                        rm -rf build-release.${DISTRO}/_deps/gtest-src build-release.${DISTRO}/_deps/gtest-build || true
                        rm -rf build-debug.${DISTRO}/_deps/gtest-src build-debug.${DISTRO}/_deps/gtest-build || true
                        rm -rf build-coverage.${DISTRO}/_deps/gtest-src build-coverage.${DISTRO}/_deps/gtest-build || true
                      '''
                    }
                  }
                }
              }
            }
          }
          stage('Build') {
            stages {
              stage('Build debug') {
                steps {
                  dir("build-debug.${env.DISTRO}") {
                    sh '''
                      rm -rf CMakeCache.txt CMakeFiles _deps || true
                      cmake -DCMAKE_BUILD_TYPE=Debug -DSTRICT=ON -DBUILD_COVERAGE=OFF \
                            -DBUILD_DOCUMENTATION=OFF -DBUILD_EXAMPLES=ON -DBUILD_TESTS=ON \
                            -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DGENERATE_PYLIBFRANKA=ON ..
                      make -j$(nproc)
                      cmake --install . --prefix ../install-debug.${DISTRO}
                    '''
                  }
                }
              }
              stage('Build release') {
                steps {
                  dir("build-release.${env.DISTRO}") {
                    sh '''
                      rm -rf CMakeCache.txt CMakeFiles _deps || true
                      cmake -DCMAKE_BUILD_TYPE=Release -DSTRICT=ON -DBUILD_COVERAGE=OFF \
                            -DBUILD_DOCUMENTATION=ON -DBUILD_EXAMPLES=ON -DBUILD_TESTS=ON \
                            -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DGENERATE_PYLIBFRANKA=ON ..
                      make -j$(nproc)
                      cmake --install . --prefix ../install-release.${DISTRO}
                    '''
                  }
                }
              }
              stage('Build examples (debug)') {
                steps {
                  script {
                    def distro = DISTRO_VERSIONS[env.UBUNTU_VERSION]
                    dir("build-debug-examples.${distro}") {
                      sh "cmake -DCMAKE_PREFIX_PATH=../install-debug.${distro} ../examples"
                      sh 'make -j$(nproc)'
                    }
                  }
                }
              }
              stage('Build examples (release)') {
                steps {
                  script {
                    def distro = DISTRO_VERSIONS[env.UBUNTU_VERSION]
                    dir("build-release-examples.${distro}") {
                      sh "cmake -DCMAKE_PREFIX_PATH=../install-release.${distro} ../examples"
                      sh 'make -j$(nproc)'
                    }
                  }
                }
              }
              stage('Build coverage') {
                when {
                  not {
                    environment name: 'UBUNTU_VERSION', value: '24.04'
                  }
                }
                steps {
                  dir("build-coverage.${env.DISTRO}") {
                    sh '''
                      rm -rf CMakeCache.txt CMakeFiles _deps || true
                      cmake -DCMAKE_BUILD_TYPE=Debug -DBUILD_COVERAGE=ON \
                            -DBUILD_DOCUMENTATION=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
                            -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=ON ..
                      make -j$(nproc)
                    '''
                  }
                }
              }
            }
          }
          stage('Lint') {
            steps {
              dir("build-lint.${env.DISTRO}") {
                catchError(buildResult: env.UNSTABLE, stageResult: env.UNSTABLE) {
                  sh '''
                    cmake -DBUILD_COVERAGE=OFF -DBUILD_DOCUMENTATION=OFF -DBUILD_EXAMPLES=ON -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DBUILD_TESTS=ON ..
                    make check-tidy -j$(nproc)
                  '''
                }
              }
            }
          }
          stage('Format') {
            steps {
              dir("build-format.${env.DISTRO}") {
                catchError(buildResult: env.UNSTABLE, stageResult: env.UNSTABLE) {
                  sh '''
                    cmake -DBUILD_COVERAGE=OFF -DBUILD_DOCUMENTATION=OFF -DBUILD_EXAMPLES=ON -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DBUILD_TESTS=ON ..
                    make check-format -j$(nproc)
                  '''
                }
              }
            }
          }
          stage('Coverage') {
            when {
              not {
                environment name: 'UBUNTU_VERSION', value: '24.04'
              }
            }
            steps {
              dir("build-coverage.${env.DISTRO}") {
                catchError(buildResult: env.UNSTABLE, stageResult: env.UNSTABLE) {
                  sh '''
                    cmake -DBUILD_COVERAGE=ON -DBUILD_DOCUMENTATION=OFF -DBUILD_EXAMPLES=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DBUILD_TESTS=ON ..
                    make coverage -j$(nproc)
                  '''
                  publishHTML([allowMissing: false,
                              alwaysLinkToLastBuild: false,
                              keepAll: true,
                              reportDir: 'coverage',
                              reportFiles: 'index.html',
                              reportName: "Code Coverage (${env.DISTRO})"])
                }
              }
            }
          }
          stage('Test') {
            steps {
              catchError(buildResult: env.UNSTABLE, stageResult: env.UNSTABLE) {
                                timeout(time: 300, unit: 'SECONDS') {
                  sh '''
                    # ASLR Fix: Disable ASLR temporarily for ASan compatibility
                    echo "[Debug Tests] Disabling ASLR for ASan compatibility..."
                    echo 0 | sudo tee /proc/sys/kernel/randomize_va_space || echo "Could not disable ASLR"
                    echo "[Debug Tests] ASLR status: $(cat /proc/sys/kernel/randomize_va_space)"
                  '''

                  script {
                    def distro = DISTRO_VERSIONS[env.UBUNTU_VERSION]
                    dir("build-debug.${distro}") {
                      sh '''
                        echo "[Debug Tests] Running tests..."
                        ctest -V
                      '''
                    }

                    dir("build-release.${distro}") {
                      sh '''
                        echo "[Release Tests] Running tests..."
                        ctest -V
                      '''
                    }
                  }

                  sh '''
                    # Re-enable ASLR for security
                    echo "[Debug Tests] Re-enabling ASLR..."
                    echo 2 | sudo tee /proc/sys/kernel/randomize_va_space || echo "Could not re-enable ASLR"
                    echo "[Debug Tests] ASLR restored to: $(cat /proc/sys/kernel/randomize_va_space)"
                  '''
                }
              }
            }
            post {
              always {
                catchError(buildResult: env.UNSTABLE, stageResult: env.UNSTABLE) {
                  junit "build-release.${DISTRO_VERSIONS[env.UBUNTU_VERSION]}/test_results/*.xml"
                  junit "build-debug.${DISTRO_VERSIONS[env.UBUNTU_VERSION]}/test_results/*.xml"
                }
              }
            }
          }
          stage('Check Github Sync') {
            steps {
              sh '.ci/checkgithistory.sh https://github.com/frankarobotics/libfranka.git develop'
            }
          }
          stage('Publish') {
            steps {
              script {
                def distro = DISTRO_VERSIONS[env.UBUNTU_VERSION]
                dir("build-release.${distro}") {
                  catchError(buildResult: env.UNSTABLE, stageResult: env.UNSTABLE) {
                    sh 'cpack'

                    // Publish Debian packages with Git commit hash in the name
                    fePublishDebian('*.deb', 'fci', "deb.distribution=${distro};deb.component=main;deb.architecture=amd64")

                    dir('doc') {
                      sh 'mv docs/*/html/ html/'
                      sh 'tar cfz ../libfranka-docs.tar.gz html'
                    }
                    sh "rename -e 's/(.tar.gz)\$/-${distro}\$1/' *.tar.gz"
                    publishHTML([allowMissing: false,
                                alwaysLinkToLastBuild: false,
                                keepAll: true,
                                reportDir: 'doc/html',
                                reportFiles: 'index.html',
                                reportName: "API Documentation (${distro})"])
                  }
                }
              }

              // Build and publish pylibfranka documentation
              catchError(buildResult: env.UNSTABLE, stageResult: env.UNSTABLE) {
                script {
                  def distro = DISTRO_VERSIONS[env.UBUNTU_VERSION]
                  withEnv(["DISTRO=${distro}"]) {
                    sh '''
                      # Install pylibfranka from root (builds against libfranka in build-release.${DISTRO})
                      export LD_LIBRARY_PATH="${WORKSPACE}/build-release.${DISTRO}:${LD_LIBRARY_PATH:-}"
                      if [ -n "$VIRTUAL_ENV" ]; then
                        python3 -m pip install .
                      else
                        python3 -m pip install . --user
                      fi
                    '''

                    dir('pylibfranka/docs') {
                      sh '''
                        # Install Sphinx and dependencies (respect virtualenv if present)
                        if [ -n "$VIRTUAL_ENV" ]; then
                          python3 -m pip install -r requirements.txt
                        else
                          # Add sphinx to PATH for --user installs
                          export PATH="$HOME/.local/bin:$PATH"
                          python3 -m pip install -r requirements.txt --user
                        fi

                        # Set locale
                        export LC_ALL=C.UTF-8
                        export LANG=C.UTF-8

                        # Add libfranka to library path
                        export LD_LIBRARY_PATH="${WORKSPACE}/build-release.${DISTRO}:${LD_LIBRARY_PATH:-}"

                        # Build the documentation only on Ubuntu 20.04
                        if [ "${UBUNTU_VERSION}" = "20.04" ]; then
                          make html
                        else
                          echo "Skipping docs build on ${UBUNTU_VERSION}"
                        fi
                      '''

                      publishHTML([allowMissing: false,
                                  alwaysLinkToLastBuild: false,
                                  keepAll: true,
                                  reportDir: '_build/html',
                                  reportFiles: 'index.html',
                                  reportName: "pylibfranka Documentation (${distro})"])
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }
  post {
    success {
      fePublishBuildInfo()
    }
    always {
      cleanWs()
      script {
        notifyBitbucket()
      }
    }
  }
}
