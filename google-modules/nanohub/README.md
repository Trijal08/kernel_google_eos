# devicetree for Eos

The code in this branch was imported from:

https://partner-android.git.corp.google.com/kernel/private/exynos-google-cw-extra/drivers/+/a30521d7964b098374a11cf534c57c84368f3c2d/sensorhub/nanohub

which is a monorepo containing all out-of-tree kernel modules for r11. To import
just the nanohub commits, [andrewevans@](http://who/andrewevans) did the
following:

## Make patches of all the commits to `nanohub`
```
cd /path/to/r11-kernel/private/exynos-google-cw-extra/drivers/sensorhub/nanohub
git format-patch --relative=sensorhub/nanohub -o /tmp/nanohub --root HEAD .
```

`--relative` rewrites paths to be rooted at the path specified,
effectively deleting that path prefix. Changes to files outside this
path are omitted from the output.

`--root` walks history starting from the oldest reachable ancestor,
i.e. the first commit. Future imports should instead specify the range
of commits since the last import.

## Switch to nanohub module repo
```
cd /path/to/eos-kernel/private/google-modules/nanohub
```

## Apply patches to Google Eos devicetree repo
```
git am /tmp/devicetree/*.patch
```

## Push the resulting changes
```
git push -o skip-validation -f -u partner HEAD:refs/heads/android-msm-eos-5.4
```
