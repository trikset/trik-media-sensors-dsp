{
    "server algorithms": {
        "programName": "bin/dsp_server.xe674",
        "algs": [
            {
                "name": "vidtranscode_cv",
                "pkg": "trik.vidtranscode_cv",
                "pkgVersion": [
                    "1, 0, 0",
                ],
                "mod": "trik.vidtranscode_cv.VIDTRANSCODE_CV",
                "threadAttrs": {
                    "priority": 2,
                },
                "groupId": 2,
                "ialgFxns": "TRIK_VIDTRANSCODE_CV_FXNS",
                "serverFxns": "VIDTRANSCODE_SKEL",
                "stubFxns": "VIDTRANSCODE_STUBS",
                "rpcProtocolVersion": 1,
            },
        ],
    },
}
