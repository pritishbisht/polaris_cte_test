pipeline {
    agent {
        kubernetes {
            yaml """
            apiVersion: v1
            kind: Pod
            metadata:
              labels:
                job: polaris_gem_simulation
            spec:
              containers:
              - name: docker
                image: docker:19-dind
                securityContext:
                  privileged: true
                command: ["dockerd-entrypoint.sh"]
                args: ["--host=tcp://0.0.0.0:2375", "--host=unix:///var/run/docker.sock"]
            """
        }
    }

    parameters {
        string(name: 'DOCKER_IMAGE', defaultValue: 'pritishbisht/polaris_gem_sim', description: 'Docker image name')
    }

    environment {
        IMAGE_NAME = "${params.DOCKER_IMAGE}"
        TAG_LATEST = "${IMAGE_NAME}:latest"
        TAG_BUILD = "${IMAGE_NAME}:build-${BUILD_NUMBER}"
        CONTAINER_NAME = "test_sim_container"
        EMAIL_RECIPIENTS = "pritishbisht123@gmail.com"
    }

    stages {
        
        stage('Checkout') {
            steps {
                container('docker') {
                    git branch: 'main', url: 'https://github.com/pritishbisht/polaris_cte_test.git'
                }
            }
        }

stage('Build or Pull Docker Image') {
            steps {
                container('docker') {
                    script {
                        // Check if Dockerfile has changed
                        def dockerfileChanged = sh(
                            script: 'git diff --name-only HEAD~1 HEAD | grep -q Dockerfile && echo "yes" || echo "no"',
                            returnStdout: true
                        ).trim() == 'yes'
                        
                        // Check if latest image exists in Docker repository
                        def imageExists = sh(
                            script: "docker manifest inspect ${TAG_LATEST} > /dev/null 2>&1 && echo 'yes' || echo 'no'",
                            returnStdout: true
                        ).trim() == 'yes'
                        
                        // Decision-making based on Dockerfile changes and image existence
                        if (dockerfileChanged || !imageExists) {
                            echo "Dockerfile changed or no existing image found. Building Docker image."
                            try {
                                sh "docker build -t ${TAG_LATEST} -t ${TAG_BUILD} ."
                            } catch (Exception e) {
                                error "Failed to build Docker image: ${e.message}"
                            }
                        } else {
                            echo "No change in Dockerfile and latest image exists. Pulling Docker image."
                            try {
                                sh "docker pull ${TAG_LATEST}"
                            } catch (Exception e) {
                                error "Failed to pull Docker image: ${e.message}"
                            }
                        }
                    }
                }
            }
        }
        
        stage('Start Container and Copy Test Package') {
            steps {
                container('docker') {
                    script {
                        // Start the container if it's not already running
                        sh 'docker run -itd --name ${CONTAINER_NAME} ${TAG_LATEST} || echo "Container already running"'

                        // Copy the test package into the running container
                        sh 'docker cp ./test_cte $CONTAINER_NAME:/root/polaris_gem_ws/src'
                    }
                }
            }
        }

        stage('Run Simulation and Evaluation') {
            steps {
                container('docker') {
                    script {
                        echo "Starting simulation in Docker container..."
                        try {
                            sh '''
                                docker run --rm $TAG_LATEST /bin/bash -c "
                                    source /opt/ros/noetic/setup.sh &&
                                    source /root/polaris_gem_ws/devel/setup.bash &&
                                    roslaunch gem_gazebo gem_gazebo_rviz.launch velodyne_points:=true"
                            '''
                        } catch (Exception e) {
                            error "Simulation failed: ${e.message}"
                        }
                    }
                }
            }
        }

        stage('Run Test Package') {
            steps {
                script {
                    // Assuming a ROS test package, source ROS setup and launch the test
                    sh 'source /opt/ros/noetic/setup.bash && roslaunch test_cte test_cte.launch'
                }
            }
        }
        
        // stage('Evaluate Results') {
        //     steps {
        //         container('docker') {
        //             script {
        //                 echo "Evaluating cross-track error results..."
        //                 try {
        //                     def logs = sh(script: 'docker logs $(docker ps -q -n=1) | grep "cross"', returnStdout: true).trim()
        //                     if (logs.contains("CTE within acceptable range")) {
        //                         currentBuild.result = 'SUCCESS'
        //                         echo 'Cross-track error is within acceptable limits.'
        //                     } else {
        //                         currentBuild.result = 'FAILURE'
        //                         echo 'Cross-track error exceeds acceptable limits.'
        //                     }
        //                 } catch (Exception e) {
        //                     error "Failed to evaluate simulation results: ${e.message}"
        //                 }
        //             }
        //         }
        //     }
        // }

        stage('Push Docker Image') {
            when {
                expression { currentBuild.result == null || currentBuild.result == 'SUCCESS' }
            }
            steps {
                container('docker') {
                    script {
                        echo "Pushing Docker image to Docker Hub: ${TAG_LATEST} and ${TAG_BUILD}"
                        
                        // Use Docker Hub token for authentication
                        withCredentials([string(credentialsId: 'dockerhub-token-id', variable: 'DOCKER_TOKEN')]) {
                            sh 'docker login -u "your_dockerhub_username" -p "$DOCKER_TOKEN"'
                        }
                 
                        try {
                            // Push both tags to Docker Hub
                            sh "docker push $TAG_LATEST"
                            sh "docker push $TAG_BUILD"
                        } catch (Exception e) {
                            error "Failed to push Docker image: ${e.message}"
                        }
                    }
                }
            }
        }
    }
    
    post {
        always {
            script {
                echo 'Performing Docker cleanup...'
                sh 'docker system prune -f'
            }
        }
        success {
            echo 'Pipeline completed successfully!'
            mail to: "${EMAIL_RECIPIENTS}",
                subject: "Jenkins Build SUCCESS - ${env.JOB_NAME} #${env.BUILD_NUMBER}",
                body: "Good news! The build completed successfully.\n\n" +
                      "Job: ${env.JOB_NAME}\n" +
                      "Build Number: ${env.BUILD_NUMBER}\n" +
                      "Status: SUCCESS\n" +
                      "Check details at: ${env.BUILD_URL}"
        }
        failure {
            echo 'Pipeline failed due to unacceptable cross-track error.'
            mail to: "${EMAIL_RECIPIENTS}",
                subject: "Jenkins Build FAILURE - ${env.JOB_NAME} #${env.BUILD_NUMBER}",
                body: "The build failed.\n\n" +
                      "Job: ${env.JOB_NAME}\n" +
                      "Build Number: ${env.BUILD_NUMBER}\n" +
                      "Status: FAILURE\n" +
                      "Error: Cross-track error exceeded acceptable limits.\n" +
                      "Check details at: ${env.BUILD_URL}"
        }
    }
}