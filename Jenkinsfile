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
            filename ".ci/Dockerfile.${env.DISTRO}"
            args '-e PATH=/opt/openrobots/bin:$PATH ' +
                '-e PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH ' +
                '-e LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH ' +
                '-e PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH ' +
                '-e CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH'
            reuseNode true
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
                      cmake -DCMAKE_BUILD_TYPE=Debug -DSTRICT=OFF -DBUILD_COVERAGE=OFF \
                            -DBUILD_DOCUMENTATION=OFF -DBUILD_EXAMPLES=ON -DBUILD_TESTS=ON ..
                      make -j$(nproc)
                    '''
                  }
                }
              }
              stage('Build release') {
                steps {
                  dir("build-release.${env.DISTRO}") {
                    sh '''
                      cmake -DCMAKE_BUILD_TYPE=Release -DSTRICT=OFF -DBUILD_COVERAGE=OFF \
                            -DBUILD_DOCUMENTATION=ON -DBUILD_EXAMPLES=ON -DBUILD_TESTS=ON ..
                      make -j$(nproc)
                    '''
                  }
                }
              }
              stage('Build examples (debug)') {
                steps {
                  dir("build-debug-examples.${env.DISTRO}") {
                    sh "cmake -DFranka_DIR:PATH=../build-debug.${DISTRO} ../examples"
                    sh 'make -j$(nproc)'
                  }
                }
              }
              stage('Build examples (release)') {
                steps {
                  dir("build-release-examples.${env.DISTRO}") {
                    sh "cmake -DFranka_DIR:PATH=../build-release.${DISTRO} ../examples"
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
                dir("build-debug.${env.DISTRO}") {
                  sh 'ctest -V'
                }
                dir("build-release.${env.DISTRO}") {
                  sh 'ctest -V'
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
              sh '.ci/checkgithistory.sh https://github.com/frankaemika/libfranka.git develop'
            }
          }
          stage('Publish') {
            steps {
              dir("build-release.${env.DISTRO}") {
                catchError(buildResult: env.UNSTABLE, stageResult: env.UNSTABLE) {
                  sh 'cpack'

                  // Get Git commit hash for package naming
                  sh "echo 'Publishing with version: ${env.VERSION}'"

                  sh '''
                    DEB_FILE=$(ls *.deb)
                    echo "Original deb file: $DEB_FILE"

                    NEW_NAME="libfranka-${VERSION}-${DISTRO}_amd64.deb"
                    echo "New deb file name: $NEW_NAME"

                    # Rename the file
                    mv "$DEB_FILE" "$NEW_NAME"
                  '''

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
