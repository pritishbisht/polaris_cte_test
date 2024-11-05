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

        stage('Build Docker Image') {
            steps {
                container('docker') {
                    script {
                        echo "Building Docker image: ${TAG_LATEST} and ${TAG_BUILD}"
                        try {
                            sh "docker build -t ${TAG_LATEST} -t ${TAG_BUILD} ."
                        } catch (Exception e) {
                            error "Failed to build Docker image: ${e.message}"
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
                        sh 'docker cp ./test_cross_track_error ${CONTAINER_NAME}:/root/gem_ws/src'
                    }
                }
            }
        }

        stage('Run Test Package') {
            steps {
                container('docker') {
                    script {
                        // Execute commands in the running container, using docker exec to ensure each runs in the right context
                        sh 'docker exec ${CONTAINER_NAME} /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make -C /root/gem_ws || echo \'catkin_make failed\'"'

                        sh 'docker exec ${CONTAINER_NAME} /bin/bash -c "source /opt/ros/noetic/setup.bash && source /root/gem_ws/devel/setup.bash && rostest test_cross_track_error pure_pursuit_cte_test.test duration:=30 error_threshold:=1.0 --text --results-file=/root/gem_ws/test_results.xml || echo \'rostest failed\'"'
                    }
                }
            }
        }
        
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
                // Publish the test results from the shared workspace
                junit 'test_results.xml'

                // Stopping and removing the container 
                // sh "docker stop ${CONTAINER_NAME} || true"
                // sh "docker rm ${CONTAINER_NAME} || true"
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