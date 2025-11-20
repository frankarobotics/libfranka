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
    timeout(time: 1, unit: 'HOURS')
  }
  environment {
    VERSION = feDetermineVersionFromGit()
    UNSTABLE = 'UNSTABLE'
  }
  stages {
    stage('Matrix') {
      matrix {
        agent {
          dockerfile {
            dir ".ci"
            filename "Dockerfile.${env.DISTRO}"
            reuseNode true
            args '--privileged ' +
                 '--cap-add=SYS_PTRACE ' +
                 '--security-opt seccomp=unconfined ' +
                 '--shm-size=2g '
         }
        }
        axes {
          axis {
            name 'DISTRO'
            values 'focal'
          }
        }
        stages {
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
                  sh "rm -rf build-*${DISTRO}"
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
                      cmake -DCMAKE_BUILD_TYPE=Debug -DSTRICT=ON -DBUILD_COVERAGE=OFF \
                            -DBUILD_DOCUMENTATION=OFF -DBUILD_EXAMPLES=ON -DBUILD_TESTS=ON \
                            -DGENERATE_PYLIBFRANKA=ON ..
                      make -j$(nproc)
                    '''
                  }
                }
              }
              stage('Build release') {
                steps {
                  dir("build-release.${env.DISTRO}") {
                    sh '''
                      cmake -DCMAKE_BUILD_TYPE=Release -DSTRICT=ON -DBUILD_COVERAGE=OFF \
                            -DBUILD_DOCUMENTATION=ON -DBUILD_EXAMPLES=ON -DBUILD_TESTS=ON \
                            -DGENERATE_PYLIBFRANKA=ON ..
                      make -j$(nproc)
                    '''
                  }
                }
              }
              stage('Build examples (debug)') {
                steps {
                  dir("build-debug-examples.${env.DISTRO}") {
                    sh "cmake -DFranka_DIR:PATH=../build-debug.${env.DISTRO} ../examples"
                    sh 'make -j$(nproc)'
                  }
                }
              }
              stage('Build examples (release)') {
                steps {
                  dir("build-release-examples.${env.DISTRO}") {
                    sh "cmake -DFranka_DIR:PATH=../build-release.${env.DISTRO} ../examples"
                    sh 'make -j$(nproc)'
                  }
                }
              }
              stage('Build coverage') {
                steps {
                  dir("build-coverage.${env.DISTRO}") {
                    sh '''
                      cmake -DCMAKE_BUILD_TYPE=Debug -DBUILD_COVERAGE=ON \
                            -DBUILD_DOCUMENTATION=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=ON ..
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
                    cmake -DBUILD_COVERAGE=OFF -DBUILD_DOCUMENTATION=OFF -DBUILD_EXAMPLES=ON -DBUILD_TESTS=ON ..
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
                    cmake -DBUILD_COVERAGE=OFF -DBUILD_DOCUMENTATION=OFF -DBUILD_EXAMPLES=ON -DBUILD_TESTS=ON ..
                    make check-format -j$(nproc)
                  '''
                }
              }
            }
          }
          stage('Coverage') {
            steps {
              dir("build-coverage.${env.DISTRO}") {
                catchError(buildResult: env.UNSTABLE, stageResult: env.UNSTABLE) {
                  sh '''
                    cmake -DBUILD_COVERAGE=ON -DBUILD_DOCUMENTATION=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=ON ..
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

                  dir("build-debug.${env.DISTRO}") {
                    sh '''
                      echo "[Debug Tests] Running tests..."
                      ctest -V
                    '''
                  }

                  dir("build-release.${env.DISTRO}") {
                    sh '''
                      echo "[Release Tests] Running tests..."
                      ctest -V
                    '''
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
                  junit "build-release.${env.DISTRO}/test_results/*.xml"
                  junit "build-debug.${env.DISTRO}/test_results/*.xml"
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
              dir("build-release.${env.DISTRO}") {
                catchError(buildResult: env.UNSTABLE, stageResult: env.UNSTABLE) {
                  sh 'cpack'

                  // Publish Debian packages with Git commit hash in the name
                  fePublishDebian('*.deb', 'fci', "deb.distribution=${env.DISTRO};deb.component=main;deb.architecture=amd64")

                  dir('doc') {
                    sh 'mv docs/*/html/ html/'
                    sh 'tar cfz ../libfranka-docs.tar.gz html'
                  }
                  sh "rename -e 's/(.tar.gz)\$/-${env.DISTRO}\$1/' *.tar.gz"
                  publishHTML([allowMissing: false,
                              alwaysLinkToLastBuild: false,
                              keepAll: true,
                              reportDir: 'doc/html',
                              reportFiles: 'index.html',
                              reportName: "API Documentation (${env.DISTRO})"])
                }
              }

              // Build and publish pylibfranka documentation
              catchError(buildResult: env.UNSTABLE, stageResult: env.UNSTABLE) {
                sh '''
                  # Install pylibfranka from root (builds against libfranka in build-release.focal)
                  export LD_LIBRARY_PATH="${WORKSPACE}/build-release.${DISTRO}:${LD_LIBRARY_PATH:-}"
                  pip3 install . --user
                '''

                dir('pylibfranka/docs') {
                  sh '''
                    # Add sphinx to PATH
                    export PATH="$HOME/.local/bin:$PATH"

                    # Install Sphinx and dependencies
                    pip3 install -r requirements.txt --user

                    # Set locale
                    export LC_ALL=C.UTF-8
                    export LANG=C.UTF-8

                    # Add libfranka to library path
                    export LD_LIBRARY_PATH="${WORKSPACE}/build-release.${DISTRO}:${LD_LIBRARY_PATH:-}"

                    # Build the documentation
                    make html
                  '''

                  publishHTML([allowMissing: false,
                              alwaysLinkToLastBuild: false,
                              keepAll: true,
                              reportDir: '_build/html',
                              reportFiles: 'index.html',
                              reportName: "pylibfranka Documentation (${env.DISTRO})"])
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
