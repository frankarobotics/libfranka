pipeline {
  libraries {
    lib('fe-pipeline-steps@1.5.0')
  }
  agent none

  triggers {
    pollSCM('H/5 * * * *')
  }
  options {
    buildDiscarder(logRotator(numToKeepStr: '30'))
    parallelsAlwaysFailFast()
    timeout(time: 1, unit: 'HOURS')
  }

  stages {
    stage('Prepare') {
      agent {
        dockerfile {
          filename './Dockerfile'
          label 'docker1'
          reuseNode true
        }
      }
      steps {
        script {
          notifyBitbucket()
          env.VERSION = feDetermineVersionFromGit()
        }
        feSetupPip()
        sh './scripts/setup_dependencies.sh'
      }
    }

    stage('Get Package Version') {
      agent {
        dockerfile {
          filename './Dockerfile'
          label 'docker1'
          reuseNode true
        }
      }
      steps {
        sh './scripts/get_version.sh'
        script {
          def props = readProperties file: 'version_info.properties'
          env.PACKAGE_VERSION = props.PACKAGE_VERSION
        }
      }
    }

    stage('Build') {
      agent {
        dockerfile {
          filename './Dockerfile'
          label 'docker1'
          reuseNode true
        }
      }
      steps {
        sh './scripts/build_package.sh'
        stash includes: 'dist/*.whl', name: 'built-wheels'
      }
    }

    stage('Vendoring') {
      agent {
        dockerfile {
          filename './Dockerfile'
          label 'docker1'
          reuseNode true
        }
      }
      steps {
        unstash 'built-wheels'
        sh './scripts/repair_wheels.sh'
        stash includes: 'wheelhouse/*.whl', name: 'wheels'
      }
    }

    stage('Test') {
      agent {
        docker {
          image 'python:3.10-slim'
          label 'docker1'
          reuseNode true
          args '-v $WORKSPACE:/workspace -e HOME=/workspace/.test_home --user root'
        }
      }
      steps {
        // Unstash the wheels from the build stage
        unstash 'wheels'
        catchError(buildResult: 'UNSTABLE', stageResult: 'UNSTABLE') {
          sh '''
            cd /workspace
            # Ensure the HOME directory exists and is writable
            mkdir -p $HOME
            mkdir -p $HOME/.cache

            # Run the test script with manylinux wheel pattern
            /workspace/scripts/test_installation.sh "-manylinux_2_34_x86_64.whl"
          '''
        }
      }
    }

    stage('Analyze') {
      parallel {
        stage('Lint') {
          agent {
            dockerfile {
              filename './Dockerfile'
              label 'docker1'
              reuseNode true
            }
          }
          steps {
            catchError(buildResult: 'UNSTABLE', stageResult: 'UNSTABLE') {
              sh './scripts/lint_code.sh'
            }
          }
        }
      }
    }

    stage('Publish') {
      agent {
        dockerfile {
          filename './Dockerfile'
          label 'docker1'
          reuseNode true
        }
      }
      steps {
        fePublishPip("wheelhouse/*.whl", "tools")
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
  }
}
