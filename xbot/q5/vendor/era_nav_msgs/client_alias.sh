#!/usr/bin/env bash

# alias for NavMapOpClient
alias LoadNavMap='python3 -m era_nav_pyclient.NavMapOpClient LoadMap'
alias SaveNavMap='python3 -m era_nav_pyclient.NavMapOpClient SaveMap'
alias ReadNavMap='python3 -m era_nav_pyclient.NavMapOpClient ReadMap'
alias ClearNavMap='python3 -m era_nav_pyclient.NavMapOpClient ClearMap'
alias StartRecordingPath='python3 -m era_nav_pyclient.NavMapOpClient StartRecordingPath'
alias MarkUserNodeOnNewPath='python3 -m era_nav_pyclient.NavMapOpClient MarkUserNodeOnNewPath'
alias FinishRecordingPath='python3 -m era_nav_pyclient.NavMapOpClient FinishRecordingPath'
alias CancelRecordingPath='python3 -m era_nav_pyclient.NavMapOpClient CancelRecordingPath'
alias RecordUserNode='python3 -m era_nav_pyclient.NavMapOpClient RecordUserNode'
alias RecordForbiddenArea='python3 -m era_nav_pyclient.NavMapOpClient RecordForbiddenArea'

# alias for NavActionClient
alias NavToGlobalNode='python3 -m era_nav_pyclient.NavActionClient NavToGlobalNode'
alias NavToGlobalPose='python3 -m era_nav_pyclient.NavActionClient NavToGlobalPose'
alias NavToLocalPose='python3 -m era_nav_pyclient.NavActionClient NavToLocalPose'
alias FollowObject='python3 -m era_nav_pyclient.NavActionClient FollowObject'
alias DockToStation='python3 -m era_nav_pyclient.NavActionClient DockToStation'
alias CancelNav='python3 -m era_nav_pyclient.NavActionClient CancelNav'

#alias for SlamClient
alias StartLidarMapping='python3 -m era_nav_pyclient.SlamClient start_map'
alias CancelLidarMapping='python3 -m era_nav_pyclient.SlamClient cancel_map'
alias CreateLidarMap='python3 -m era_nav_pyclient.SlamClient create_map'
alias LoadLidarMap='python3 -m era_nav_pyclient.SlamClient load_map'
alias InitLidarPos='python3 -m era_nav_pyclient.SlamClient init_pos'
alias QueryLidarMap='python3 -m era_nav_pyclient.SlamClient query_map'
alias MonitorSlamState='python3 -m era_nav_pyclient.SlamClient'
