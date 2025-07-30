#!groovy

@Library('PSL')

def jobs = [:]

node('aws-linux')
{
    stage('Harmony Scan')
    {
        scan = new ors.security.CommonHarmony(steps, env, Artifactory, scm).run_scan([
            "repository":"Research/Inukshuk",
            "product_output":"${env.WORKSPACE}",
            "analyze_results": true
        ])
    }
}

parallel jobs
